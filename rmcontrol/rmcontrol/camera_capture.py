"""Remote RealSense capture helpers shared by camera_recorder and test.py."""

CPP = r'''
#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <string>
int main(int argc,char**argv) {
 try {
  rs2::context ctx;
  auto devices=ctx.query_devices();
  if(!devices.size()) throw std::runtime_error("No RealSense device found");
  for(size_t i=0;i<devices.size();++i) {
   auto dev=devices[i];
   std::string serial=dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
   std::string dir=std::string(argv[1])+"/camera_"+std::to_string(i);
   rs2::pipeline pipe(ctx); rs2::config cfg;
   cfg.enable_device(serial);
   cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
   cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_RGB8,30);
   auto profile=pipe.start(cfg);
   rs2::frameset frames;
   for(int n=0;n<30;++n) frames=pipe.wait_for_frames(10000);
   rs2::align align(RS2_STREAM_COLOR);
   frames=align.process(frames);
   auto color=frames.get_color_frame(); auto depth=frames.get_depth_frame();
   if(!color || !depth) throw std::runtime_error("Missing color/depth frame");
   std::ofstream rgb(dir+"_color.ppm",std::ios::binary);
   rgb<<"P6\n"<<color.get_width()<<" "<<color.get_height()<<"\n255\n";
   for(int y=0;y<color.get_height();++y)
    rgb.write((const char*)color.get_data()+y*color.get_stride_in_bytes(),color.get_width()*3);
   std::ofstream z(dir+"_depth.pgm",std::ios::binary);
   z<<"P5\n"<<depth.get_width()<<" "<<depth.get_height()<<"\n65535\n";
   for(int y=0;y<depth.get_height();++y) {
    auto row=(const uint16_t*)((const char*)depth.get_data()+y*depth.get_stride_in_bytes());
    for(int x=0;x<depth.get_width();++x) {z.put(row[x]>>8); z.put(row[x]&255);}
   }
   auto intr=depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
   std::ofstream meta(dir+"_metadata.json");
   meta<<"{\"serial\":\""<<serial<<"\",\"model\":\""<<dev.get_info(RS2_CAMERA_INFO_NAME)
       <<"\",\"depth_scale_m\":"<<depth.get_units()
       <<",\"depth_aligned_to_color\":true,\"invalid_depth_value\":0"
       <<",\"width\":"<<intr.width<<",\"height\":"<<intr.height
       <<",\"fx\":"<<intr.fx<<",\"fy\":"<<intr.fy
       <<",\"cx\":"<<intr.ppx<<",\"cy\":"<<intr.ppy<<"}\n";
   pipe.stop();
   std::cout<<"Captured "<<dev.get_info(RS2_CAMERA_INFO_NAME)<<" serial="<<serial<<std::endl;
  }
 } catch(const std::exception&e) {std::cerr<<e.what()<<std::endl;return 1;}
}
'''

CONVERT = r'''
import cv2, glob, json, numpy as np, os, sys
for path in glob.glob(sys.argv[1]+'/*_color.ppm'):
    prefix=path[:-10]
    color=cv2.imread(path)
    depth=cv2.imread(prefix+'_depth.pgm',cv2.IMREAD_UNCHANGED)
    with open(prefix+'_metadata.json') as f: meta=json.load(f)
    meters=depth.astype(np.float32)*meta['depth_scale_m']
    # 固定 0~5 米颜色范围；无效深度为黑色。原始 PNG 保留真实数值。
    preview=cv2.applyColorMap((np.clip(meters/5,0,1)*255).astype(np.uint8),cv2.COLORMAP_JET)
    preview[depth==0]=0
    for suffix,im in [('_color.png',color),('_depth.png',depth),('_depth_preview.png',preview)]:
        if not cv2.imwrite(prefix+suffix,im): raise RuntimeError('Cannot save '+suffix)
    print(os.path.basename(prefix), 'valid depth pixels:', int((depth>0).sum()))
'''

# stdout is a framed binary protocol; camera pipelines stay open between requests.
STREAM_CPP = r'''
#include <librealsense2/rs.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <memory>
#include <iomanip>
#include <opencv2/opencv.hpp>
struct Camera {
 rs2::pipeline pipe;
 std::string serial, model;
 Camera(rs2::context& ctx, rs2::device dev):pipe(ctx) {
  serial=dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  model=dev.get_info(RS2_CAMERA_INFO_NAME);
  rs2::config cfg; cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
  cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_RGB8,30);
  pipe.start(cfg);
  for(int n=0;n<30;++n) pipe.wait_for_frames(10000);
 }
};
int main() {
 try {
  rs2::context ctx; std::vector<std::unique_ptr<Camera>> cameras;
  for(auto dev:ctx.query_devices()) cameras.emplace_back(new Camera(ctx,dev));
  if(cameras.empty()) throw std::runtime_error("No RealSense device found");
  std::cout<<"READY "<<cameras.size()<<std::endl;
  std::string command;
  while(std::getline(std::cin,command) && command=="capture") {
   for(auto& camera:cameras) {
    // A bounded queue discards old frames while waiting for the next request.
    auto frames=camera->pipe.wait_for_frames(10000);
    rs2::frameset latest;
    for(int n=0;n<10 && camera->pipe.poll_for_frames(&latest);++n) frames=latest;
    double source_ms=frames.get_depth_frame().get_timestamp();
    int domain=(int)frames.get_depth_frame().get_frame_timestamp_domain();
    auto frame_number=frames.get_depth_frame().get_frame_number();
    rs2::align align(RS2_STREAM_COLOR); frames=align.process(frames);
    auto color=frames.get_color_frame(); auto depth=frames.get_depth_frame();
    if(!color || !depth) throw std::runtime_error("Missing color/depth frame");
    auto intr=depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    cv::Mat rgb(color.get_height(),color.get_width(),CV_8UC3,
                (void*)color.get_data(),color.get_stride_in_bytes());
    cv::Mat bgr; cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);
    cv::Mat z(depth.get_height(),depth.get_width(),CV_16UC1,
              (void*)depth.get_data(),depth.get_stride_in_bytes());
    std::vector<unsigned char> jpg,png;
    if(!cv::imencode(".jpg",bgr,jpg,{cv::IMWRITE_JPEG_QUALITY,95}) ||
       !cv::imencode(".png",z,png,{cv::IMWRITE_PNG_COMPRESSION,3}))
       throw std::runtime_error("Image encoding failed");
    std::ostringstream header;
    header<<std::setprecision(12)
     <<"{\"serial\":\""<<camera->serial<<"\",\"model\":\""<<camera->model
     <<"\",\"width\":"<<intr.width<<",\"height\":"<<intr.height
     <<",\"depth_scale_m\":"<<depth.get_units()
     <<",\"depth_aligned_to_color\":true,\"invalid_depth_value\":0"
     <<",\"fx\":"<<intr.fx<<",\"fy\":"<<intr.fy
     <<",\"cx\":"<<intr.ppx<<",\"cy\":"<<intr.ppy
     <<",\"distortion_model\":"<<(int)intr.model<<",\"distortion_coefficients\":[";
    for(int i=0;i<5;++i) header<<(i?",":"")<<intr.coeffs[i];
    header<<"],\"source_timestamp_ms\":"<<source_ms
     <<",\"source_timestamp_domain\":"<<domain
     <<",\"source_frame_number\":"<<frame_number
     <<",\"color_jpeg_bytes\":"<<jpg.size()<<",\"depth_png_bytes\":"<<png.size()
     <<",\"color_jpeg_quality\":95}";
    std::cout<<header.str()<<'\n';
    std::cout.write((const char*)jpg.data(),jpg.size());
    std::cout.write((const char*)png.data(),png.size());
    std::cout.flush();
   }
  }
 } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
'''
