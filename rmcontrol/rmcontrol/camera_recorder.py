"""Record both dogs' RealSense images over SSH at 1 Hz, without motion control."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import shlex
import threading
import time

from .camera_capture import STREAM_CPP


def wall_time():
    return datetime.now().astimezone().isoformat()


def run_remote(client, command):
    stdin, stdout, stderr = client.exec_command(command, timeout=60)
    stdin.close()
    try:
        output = stdout.read().decode(errors='replace')
        error = stderr.read().decode(errors='replace')
        if stdout.channel.recv_exit_status():
            raise RuntimeError(error.strip() or output.strip() or 'Remote command failed')
        return output.strip()
    finally:
        stdout.close()
        stderr.close()


def read_exact(stream, size):
    result = bytearray()
    while len(result) < size:
        part = stream.read(size - len(result))
        if not part:
            raise RuntimeError('相机数据流意外结束')
        result.extend(part)
    return result


def record_host(args, host, folder, stop, log):
    import cv2
    import numpy as np
    import paramiko

    saved = 0
    while not stop.is_set() and (not args.count or saved < args.count):
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        sftp = None
        channel = None
        remote = None
        try:
            log(host, 'connecting')
            client.connect(host, username=args.user, password=args.password,
                           timeout=5, auth_timeout=5, banner_timeout=5,
                           look_for_keys=False, allow_agent=False)
            client.get_transport().set_keepalive(10)
            remote = run_remote(client, 'mktemp -d /tmp/unitree-recorder-XXXXXX')
            sftp = client.open_sftp()
            with sftp.file(remote + '/capture.cpp', 'w') as handle:
                handle.write(STREAM_CPP)
            q = shlex.quote(remote)
            run_remote(client, f'g++ -O2 -std=c++11 {q}/capture.cpp -o {q}/capture '
                               '$(pkg-config --cflags --libs realsense2 opencv4)')
            channel = client.get_transport().open_session()
            channel.settimeout(15)
            # exec preserves the process identity; closing SSH terminates this capture process.
            channel.exec_command(f'exec env LD_LIBRARY_PATH=/usr/local/lib {q}/capture')
            reader = channel.makefile('rb')
            ready = reader.readline().decode().strip()
            if not ready.startswith('READY '):
                error = channel.recv_stderr(4096).decode(errors='replace')
                raise RuntimeError(error or '相机初始化失败: ' + ready)
            cameras = int(ready.split()[1])
            log(host, 'ready', cameras=cameras, interval_s=args.interval)
            deadline = time.monotonic()
            while not stop.is_set() and (not args.count or saved < args.count):
                if stop.wait(max(0, deadline - time.monotonic())):
                    break
                request_ns = time.time_ns()
                request_mono = time.monotonic_ns()
                channel.sendall(b'capture\n')
                stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
                for index in range(cameras):
                    line = reader.readline()
                    if not line:
                        error = channel.recv_stderr(4096).decode(errors='replace')
                        raise RuntimeError(error or '相机流中断')
                    meta = json.loads(line)
                    width, height = meta['width'], meta['height']
                    if not (0 < width <= 4096 and 0 < height <= 4096):
                        raise RuntimeError('Invalid image dimensions')
                    color_size, depth_size = meta['color_jpeg_bytes'], meta['depth_png_bytes']
                    if not (0 < color_size <= 32_000_000 and 0 < depth_size <= 32_000_000):
                        raise RuntimeError('Invalid encoded image sizes')
                    color_bytes = read_exact(reader, color_size)
                    depth_bytes = read_exact(reader, depth_size)
                    depth = cv2.imdecode(np.frombuffer(depth_bytes, np.uint8), cv2.IMREAD_UNCHANGED)
                    if depth is None or depth.dtype != np.uint16 or depth.shape != (height, width):
                        raise RuntimeError('Invalid depth PNG')
                    receive_ns = time.time_ns()
                    meters = depth.astype(np.float32) * meta['depth_scale_m']
                    preview = cv2.applyColorMap(
                        (np.clip(meters / 5, 0, 1)*255).astype(np.uint8), cv2.COLORMAP_JET)
                    preview[depth == 0] = 0
                    base = folder / 'cameras' / host / ('camera_' + meta['serial'])
                    base.mkdir(parents=True, exist_ok=True)
                    target = base / stamp
                    temporary = base / ('.' + stamp + '.partial')
                    temporary.mkdir()
                    try:
                        (temporary / 'color.jpg').write_bytes(color_bytes)
                        (temporary / 'depth.png').write_bytes(depth_bytes)
                        if not cv2.imwrite(str(temporary / 'depth_preview.png'), preview):
                            raise RuntimeError('Cannot save depth preview')
                        meta.update(host=host, camera_index=index, request_unix_ns=request_ns,
                                    request_monotonic_ns=request_mono, receive_unix_ns=receive_ns,
                                    receive_monotonic_ns=time.monotonic_ns(),
                                    valid_depth_pixels=int(np.count_nonzero(depth)),
                                    preview_range_m=[0, 5])
                        (temporary / 'metadata.json').write_text(
                            json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8')
                        temporary.rename(target)
                    finally:
                        if temporary.exists():
                            shutil.rmtree(temporary)
                    log(host, 'frame', path=str(target.relative_to(folder)),
                        request_unix_ns=request_ns, receive_unix_ns=receive_ns,
                        valid_depth_pixels=meta['valid_depth_pixels'])
                saved += 1
                deadline += args.interval
                if deadline < time.monotonic():
                    log(host, 'slow_capture', elapsed_s=(time.monotonic_ns()-request_mono)/1e9)
                    deadline = time.monotonic()
        except Exception as exc:
            if not stop.is_set():
                log(host, 'error', message=str(exc), retry_s=args.retry_interval)
        finally:
            if channel is not None:
                channel.close()
            if sftp is not None:
                # Only remove the temporary sources and binary created by this connection.
                if remote:
                    try:
                        for name in ('capture.cpp', 'capture'):
                            sftp.remove(remote + '/' + name)
                        sftp.rmdir(remote)
                    except OSError:
                        pass
                sftp.close()
            client.close()
        if not stop.is_set() and (not args.count or saved < args.count):
            stop.wait(args.retry_interval)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hosts', nargs='+', default=['172.20.10.13', '172.20.10.14'])
    parser.add_argument('--user', default='unitree')
    parser.add_argument('--password', default='123')
    parser.add_argument('--interval', type=float, default=1.0, help='采集间隔，秒（默认1）')
    parser.add_argument('--retry-interval', type=float, default=10.0)
    parser.add_argument('--output-dir', type=Path, default=Path('run_data'))
    parser.add_argument('--count', type=int, default=0, help='每台狗采集次数，0表示持续记录')
    config = parser.parse_args(args)
    if (not math.isfinite(config.interval) or config.interval <= 0 or
            not math.isfinite(config.retry_interval) or config.retry_interval <= 0 or
            config.count < 0):
        parser.error('interval/retry-interval 必须为正有限数，count 必须非负')
    try:
        import paramiko  # noqa: F401
        import cv2  # noqa: F401
        import numpy  # noqa: F401
    except ImportError as exc:
        parser.error(f'缺少依赖: {exc}; 请安装 python3-paramiko python3-opencv python3-numpy')
    config.hosts = list(dict.fromkeys(config.hosts))
    folder = config.output_dir.expanduser().resolve() / datetime.now().astimezone().strftime(
        '%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    metadata = dict(started_at=wall_time(), hosts=config.hosts, interval_s=config.interval,
                    format='color JPEG (quality 95) + lossless uint16 depth PNG + preview PNG + JSON',
                    notes=['Depth meters = pixel value * depth_scale_m; zero is invalid.',
                           'Depth aligned to color; preview clipped to 0..5 meters.',
                           'Request/receive timestamps use recorder clock; source timestamp domain '
                           'comes from RealSense and is not necessarily Unix time.',
                           'Hosts run independently; no cross-host frame synchronization.',
                           'Camera streams run at 30 FPS; saved snapshots target 1/interval Hz.',
                           'This folder is separate from data_recorder; correlate by Unix timestamps.'])
    meta_path = folder / 'camera_recording.json'
    meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    stop = threading.Event()
    lock = threading.Lock()
    with (folder / 'camera_events.jsonl').open('a', encoding='utf-8', buffering=1) as events:
        def log(host, event, **details):
            row = dict(time=wall_time(), unix_ns=time.time_ns(), host=host, event=event, **details)
            with lock:
                events.write(json.dumps(row, ensure_ascii=False) + '\n')
                print(f'[{host}] {event}: {details}', flush=True)
        workers = [threading.Thread(target=record_host,
                                    args=(config, host, folder, stop, log)) for host in config.hosts]
        print(f'Camera recording ready: {folder}', flush=True)
        try:
            for worker in workers:
                worker.start()
            while any(worker.is_alive() for worker in workers):
                time.sleep(0.2)
        except KeyboardInterrupt:
            print('正在停止相机采集……', flush=True)
        finally:
            stop.set()
            for worker in workers:
                if worker.ident is not None:
                    worker.join()
            metadata['stopped_at'] = wall_time()
            meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
