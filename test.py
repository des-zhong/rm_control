#!/usr/bin/env python3
"""SSH 采集机器狗 RealSense 彩色图、原始深度及深度预览。
运行: python3 test.py（本机依赖 paramiko；默认密码123，可用 UNITREE_PASSWORD 覆盖）
原始 depth.png 是 uint16，距离(米) = 像素值 * metadata.json 的 depth_scale_m。
使用设备现有 librealsense，不安装软件、不启动运动控制。
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import sys
from datetime import datetime
sys.path.insert(0, str(Path(__file__).resolve().parent / "rmcontrol"))
from rmcontrol.camera_capture import CPP, CONVERT



def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='172.20.10.13')
    p.add_argument('--user',default='unitree')
    p.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'camera_captures')
    args=p.parse_args()
    try:
        import paramiko
    except ImportError:
        p.error('请先安装依赖: python3 -m pip install paramiko')
    password=os.environ.get('UNITREE_PASSWORD', '123')
    client=paramiko.SSHClient()
    client.load_system_host_keys()
    # 首次连接采用与本项目相同的自动接受策略；已有主机密钥不匹配仍会报错。
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    sftp=None
    def run(command,timeout=90):
        stdin,stdout,stderr=client.exec_command(command,timeout=timeout)
        stdin.close()
        out=stdout.read().decode(errors='replace');err=stderr.read().decode(errors='replace')
        status=stdout.channel.recv_exit_status()
        stdout.close();stderr.close()
        if out: print(out,end='')
        if status: raise RuntimeError(f'远程命令失败 ({status}): {err.strip()}')
        return out.strip()
    try:
        client.connect(args.host,username=args.user,password=password,timeout=10,auth_timeout=10)
        remote=run('mktemp -d /tmp/unitree-camera-XXXXXX')
        sftp=client.open_sftp()
        for name,content in [('capture.cpp',CPP),('convert.py',CONVERT)]:
            with sftp.file(remote+'/'+name,'w') as f: f.write(content)
        q=shlex.quote(remote)
        print('使用机器狗已有 librealsense 采集所有可见 RealSense 设备……')
        run(f'g++ -std=c++11 {q}/capture.cpp -o {q}/capture $(pkg-config --cflags --libs realsense2)')
        run(f'timeout 65 env LD_LIBRARY_PATH=/usr/local/lib {q}/capture {q}',timeout=75)
        run(f'python3 {q}/convert.py {q}')
        local=args.output/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        local.mkdir(parents=True)
        for name in sftp.listdir(remote):
            if name.endswith(('.png','.json')):
                sftp.get(remote+'/'+name,str(local/name))
        metadata=list(local.glob('*_metadata.json'))
        if not metadata: raise RuntimeError('没有生成相机数据')
        for path in metadata:
            meta=json.loads(path.read_text())
            print(f"相机: {meta['model']}，序列号: {meta['serial']}，深度单位: {meta['depth_scale_m']} 米")
        print(f'图片和深度元数据已保存至: {local}')
        print('depth.png 为原始16位深度；depth_preview.png 为0~5米伪彩色预览，0值表示无效深度。')
        print(f'远程采集文件保留于: {remote}')
    finally:
        if sftp: sftp.close()
        client.close()

if __name__=='__main__':
    try: main()
    except (Exception,KeyboardInterrupt) as exc:
        print(f'采集失败: {exc}',file=sys.stderr)
        sys.exit(1)
