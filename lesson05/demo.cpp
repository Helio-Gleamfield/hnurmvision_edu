#include "aiming.hpp"
#include "armor_detector.hpp"
#include "evaluation.hpp"
#include "io_context.hpp"
#include "lessons.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
volatile std::sig_atomic_t stopped=0;
void stop(int){stopped=1;}
// Evaluation subscriber is isolated from detector, tracking and aiming inputs.
struct ScoreMonitor {
  rclcpp::Context::SharedPtr context=std::make_shared<rclcpp::Context>();
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  YAML::Node score;
  std::ofstream history;
  training::TimePoint received{};
  ScoreMonitor(const std::string &path):history(path) {
    context->init(0,nullptr);
    node=std::make_shared<rclcpp::Node>("lesson05_score_monitor",rclcpp::NodeOptions().context(context));
    rclcpp::ExecutorOptions options;options.context=context;
    executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
    executor->add_node(node);
    subscription=node->create_subscription<std_msgs::msg::String>("/lesson04/ground_truth",rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr m){
        try {
          auto root=YAML::Load(m->data);
          if (!root["scoreboard"])return;
          score=YAML::Clone(root["scoreboard"]);received=training::Clock::now();
          // Original simulator JSON retained for independent auditing.
          history<<m->data<<'\n';
        } catch(const YAML::Exception &e){std::cerr<<"score decode: "<<e.what()<<'\n';}
      });
  }
  ~ScoreMonitor(){context->shutdown("score monitor complete");}
  void poll(){executor->spin_some(std::chrono::milliseconds(1));}
};
}
int main(int argc,char **argv) try {
  using namespace training;
  std::cout<<std::unitbuf;
  std::string out,scenario="static";double seconds=30;bool show=true;
  for(int i=1;i<argc;++i){
    std::string a=argv[i];
    auto value=[&](){if(++i>=argc)throw std::runtime_error("missing option value");return std::string(argv[i]);};
    if(a=="--output")out=value();else if(a=="--seconds")seconds=std::stod(value());
    else if(a=="--scenario")scenario=value();else if(a=="--headless")show=false;
    else if(a=="--help"){
      std::cout<<"Usage: lesson05_demo --output /fresh/directory [--seconds 32] [--scenario static|translation] [--headless]\n"
                 "Runs the simulation firing demo and saves actual camera frames, commands and independent scores.\n";
      return 0;
    }
    else throw std::runtime_error("unknown argument "+a);
  }
  if(out.empty() || std::filesystem::exists(out) || !std::isfinite(seconds) || seconds<5 || seconds>600 ||
     (scenario!="static" && scenario!="translation"))throw std::runtime_error("use a fresh --output, seconds 5..600, scenario static/translation");
  std::filesystem::create_directories(out+"/frames");
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  auto config=load_config("configs/simulation.yaml");config.allow_fire=true;
  const auto calibration=load_calibration(config);
  io::IoContext devices(config);devices.open();
  TrackingState tracking;tracking.parameters.use_tilt_prior=true;
  FireControl fire;AimParameters parameters;
  Lesson04Evaluation evaluation(out+"/evaluation");
  ScoreMonitor monitor(out+"/simulator-truth.jsonl");
  std::ofstream frames(out+"/frames.csv"),commands(out+"/commands.csv");
  frames<<"filename,acquisition_ros_ns\n";
  commands<<"acquisition_ros_ns,control,shoot,x,y,z,vx,vy,vz,age_s,flight_s,lead_s,iterations,yaw_rad,pitch_internal_rad,pitch_wire_deg,yaw_error_rad,pitch_error_rad,reason\n";
  std::optional<TimePoint> first;
  const auto start=Clock::now();auto last_input=start;
  std::int64_t last_record=0;std::size_t frame_count=0,records=0,requests=0,solutions=0;
  if(show){cv::namedWindow("Lesson05 Ballistics + Live Hits",cv::WINDOW_NORMAL);cv::resizeWindow("Lesson05 Ballistics + Live Hits",1152,864);}
  while(!stopped){
    monitor.poll();evaluation.poll();
    auto input=devices.read();
    if(input){
      last_input=Clock::now();if(!first)first=input->timestamp;
      const double elapsed=std::chrono::duration<double>(input->timestamp-*first).count();
      auto armors=lesson03_detection(input->image);
      auto target=lesson04_tracking(armors,input->timestamp,input->imu_q,calibration,tracking);
      evaluation.observe(*input,tracking,calibration);
      AimSolution aim;Command command;
      if(target){
        aim=solve_aim(*target,input->timestamp,input->bullet_speed,input->imu_q,parameters);
        command=fire.update(aim,input->imu_q,
          tracking.phase==TrackingState::Phase::Tracking && bool(tracking.observation),tracking.generation);
      } else fire.reset();
      // End with a no-fire drain so every short-range projectile has time to resolve.
      if(elapsed>=seconds-2){command.shoot=false;fire.reason="HOLD: final projectile drain";}
      devices.write(command);++frame_count;requests+=command.shoot;solutions+=command.control;
      const Target t=target.value_or(Target{});
      commands<<input->acquisition_ros_ns<<','<<command.control<<','<<command.shoot<<','
        <<t.position.x()<<','<<t.position.y()<<','<<t.position.z()<<','<<t.velocity.x()<<','<<t.velocity.y()<<','<<t.velocity.z()<<','
        <<aim.observation_age_s<<','<<aim.flight_s<<','<<aim.lead_s<<','<<aim.iterations<<','
        <<command.yaw<<','<<command.pitch<<','<<-command.pitch*180/kPi<<','
        <<fire.yaw_error<<','<<fire.pitch_error<<','<<fire.reason<<'\n';
      cv::Mat canvas=draw_armors(input->image,armors);
      if(aim.command.control){
        Eigen::Vector3d c=calibration.camera_to_gimbal.transpose()*(input->imu_q.conjugate()*aim.intercept-calibration.camera_to_gimbal_translation);
        if(c.z()>.1){
          std::vector<cv::Point2d> uv;
          cv::projectPoints(std::vector<cv::Point3d>{{c.x(),c.y(),c.z()}},cv::Vec3d(),cv::Vec3d(),calibration.matrix,calibration.distortion,uv);
          cv::drawMarker(canvas,uv[0],{255,0,255},cv::MARKER_CROSS,30,3);
        }
      }
      cv::rectangle(canvas,{0,0,1440,245},{18,23,30},cv::FILLED);
      cv::rectangle(canvas,{0,855,1440,225},{18,23,30},cv::FILLED);
      int y=36;
      auto text=[&](const std::string &s,cv::Scalar color=cv::Scalar(240,240,240)){
        cv::putText(canvas,s,{20,y},cv::FONT_HERSHEY_SIMPLEX,.76,color,2,cv::LINE_AA);y+=36;
      };
      text("LESSON 05 | "+scenario+" target | OpenCV -> PnP -> KF -> ballistics -> IO",{120,230,255});
      text(cv::format("REC  %.1f s remaining | bullet %.0f m/s | gravity 9.81 m/s^2 | delay 0 ms",std::max(0.,seconds-elapsed),input->bullet_speed));
      text(cv::format("Measured image age %.1f ms + flight %.1f ms = lead %.1f ms | iterations %d",aim.observation_age_s*1000,aim.flight_s*1000,aim.lead_s*1000,aim.iterations));
      text(cv::format("World yaw %.2f deg | wire pitch (25 - elevation) %.2f deg",command.yaw*180/kPi,-command.pitch*180/kPi));
      text("GREEN: observed armor | MAGENTA cross: predicted intercept",{255,150,255});
      text(fire.reason,command.shoot?cv::Scalar(80,255,120):cv::Scalar(0,200,255));
      y=894;
      const bool fresh=monitor.score && std::chrono::duration<double>(Clock::now()-monitor.received).count()<.5;
      if(fresh){
        int selected=monitor.score["selected"].as<int>();auto score=monitor.score["scores"][selected];
        auto shots=score["shots"].as<unsigned long long>(),hits=score["hits"].as<unsigned long long>();
        text(cv::format("SIMULATOR SCORE | shots %llu | armor hits %llu | accuracy %.1f %%",shots,hits,shots?100.*hits/shots:0),{80,255,180});
        text(cv::format("Damage %.0f (10 per hit) | off-target %llu | simulator time %.1f s",score["damage"].as<double>(),score["off_target"].as<unsigned long long>(),score["seconds"].as<double>()));
      }else{text("SIMULATOR SCORE: waiting for fresh independent telemetry",{0,180,255});y+=36;}
      text(cv::format("Feedback error: yaw %.2f deg | pitch %.2f deg | tracking %s",fire.yaw_error*180/kPi,fire.pitch_error*180/kPi,tracking_phase_name(tracking.phase)));
      text("Hit counts come from simulator projectile/armor collisions; no truth used for aiming.");
      text("Press q / Esc to stop. Recording saves automatically to the selected output folder.",{180,190,205});
      if(input->acquisition_ros_ns-last_record>=50000000LL){
        const std::string name=cv::format("%06zu.jpg",records++);
        cv::imwrite(out+"/frames/"+name,canvas,{cv::IMWRITE_JPEG_QUALITY,92});
        frames<<name<<','<<input->acquisition_ros_ns<<'\n';last_record=input->acquisition_ros_ns;
      }
      if(show)cv::imshow("Lesson05 Ballistics + Live Hits",canvas);
      if(elapsed>=seconds)break;
    }
    if(std::chrono::duration<double>(Clock::now()-last_input).count()>5 && first)throw std::runtime_error("camera stream stalled");
    if(!first && std::chrono::duration<double>(Clock::now()-start).count()>25)throw std::runtime_error("no synchronized camera");
    if(std::chrono::duration<double>(Clock::now()-last_input).count()>.2){devices.write(Command{});fire.reset();}
    if(show){int k=cv::waitKey(1);if(k=='q'||k==27)break;}
  }
  devices.close();monitor.poll();evaluation.poll();
  YAML::Node summary;
  summary["scenario"]=scenario;summary["frames"]=frame_count;summary["solution_frames"]=solutions;
  summary["fire_request_frames"]=requests;summary["simulator_scoreboard"]=monitor.score;
  summary["source"]="Actual simulator scoring, independent of vision/aiming";
  summary["score_fresh"]=monitor.score && std::chrono::duration<double>(Clock::now()-monitor.received).count()<.5;
  std::ofstream(out+"/summary.yaml")<<summary;
  evaluation.write_summary(out+"/pose-summary.yaml");
  if(show)cv::destroyAllWindows();
  std::cout<<summary<<'\n';
  if(!monitor.score || !summary["score_fresh"].as<bool>())return 2;
  auto score=monitor.score["scores"][monitor.score["selected"].as<int>()];
  return score["hits"].as<unsigned>()>0 && frame_count>0?0:2;
}catch(const std::exception &e){std::cerr<<"lesson05: "<<e.what()<<'\n';return 1;}
