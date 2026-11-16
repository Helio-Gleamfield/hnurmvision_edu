#include "aiming.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace training;
int main() try {
  unsigned checks=0;
  auto check=[&](bool ok,const char *message){++checks;if(!ok)throw std::runtime_error(message);};
  for(double d:{.1,1.,3.,12.,30.}) for(double h:{-2.,0.,1.,3.}) {
    auto b=solve_ballistic(25,d,h);check(bool(b),"reachable target");
    double x=25*std::cos(b->elevation)*b->flight_s;
    double z=25*std::sin(b->elevation)*b->flight_s-4.905*b->flight_s*b->flight_s;
    check(std::abs(x-d)<1e-10 && std::abs(z-h)<1e-9,"physical endpoint");
    check(b->flight_s<2.,"low-arc flight");
  }
  check(!solve_ballistic(5,100,0),"unreachable");
  check(!solve_ballistic(0,3,0),"zero speed");
  check(!solve_ballistic(25,0,0),"zero distance");
  check(!solve_ballistic(25,3,std::numeric_limits<double>::quiet_NaN()),"NaN");
  auto now=Clock::now();Target t;t.position={3,0,.1};t.velocity={0,.8,0};
  AimParameters p;p.muzzle_gimbal.setZero();
  auto s=solve_aim(t,now-std::chrono::milliseconds(8),25,Eigen::Quaterniond::Identity(),p,now);
  check(s.command.control && !s.command.shoot && s.iterations>=2,"iterative solution");
  check(std::abs(s.lead_s-(.008+s.flight_s))<p.convergence_s,"lead excludes gimbal lag");
  check(std::abs(s.intercept.y()-.8*s.lead_s)<1e-10,"CV intercept");
  check(std::abs(s.command.pitch-(s.elevation-25*kPi/180))<1e-12,"25deg and wire sign");
  p.delay_s=.11;
  auto later=solve_aim(t,now,25,Eigen::Quaterniond::Identity(),p,now);
  check(later.intercept.y()>s.intercept.y()+.07,"actuation delay parameter");
  check(!solve_aim(t,now-std::chrono::seconds(1),25,Eigen::Quaterniond::Identity(),p,now).command.control,"stale");
  check(!solve_aim(t,now+std::chrono::seconds(1),25,Eigen::Quaterniond::Identity(),p,now).command.control,"future");
  p.max_iterations=1;
  check(!solve_aim(t,now,25,Eigen::Quaterniond::Identity(),p,now).command.control,"nonconvergence");
  FireControl f;
  Eigen::Quaterniond q(Eigen::AngleAxisd(s.command.yaw,Eigen::Vector3d::UnitZ())*
                       Eigen::AngleAxisd(-s.command.pitch,Eigen::Vector3d::UnitY()));
  check(!f.update(s,q,true,1).shoot,"first command holds");
  check(f.update(s,q,true,1).shoot,"aligned feedback fires");
  check(!f.update(s,Eigen::Quaterniond::Identity(),true,1).shoot,"moving gimbal holds");
  check(!f.update(s,q,false,1).control,"untrusted target stops");
  check(!f.update(s,q,true,1).shoot,"reacquisition holds");
  check(!f.update(s,q,true,2).shoot,"new generation holds");
  auto wrap=s;wrap.command.yaw=kPi-.001;wrap.command.pitch=0;
  Eigen::Quaterniond qw(Eigen::AngleAxisd(-kPi+.001,Eigen::Vector3d::UnitZ()));
  f.reset();f.update(wrap,qw,true,3);check(f.update(wrap,qw,true,3).shoot,"yaw wraps");
  check(!f.update(AimSolution{},q,true,3).control,"no solution stops");
  std::cout<<"PASS: "<<checks<<" ballistic, timing and fire-control checks\n";
  return 0;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
