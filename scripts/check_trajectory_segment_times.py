"""Exercise the actual C++ segment-time block with Eigen's NaN initialization.

This isolates undefined reads; it does not model B-spline feasibility or prove
that a historical trajectory was caused by the exposed uninitialized values.
Run where the compiler and Eigen headers are installed (the ROS container).
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile

p=argparse.ArgumentParser();p.add_argument('source',type=Path)
p.add_argument('--report',type=Path);a=p.parse_args()
s=a.source.read_text();begin=s.index('  const int pt_num = tour.size();')
end=s.index('  // Print times',begin);block=s[begin:end].split('  std::string timing_values=')[0]
program='''#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;
struct Limits { double max_vel_=2.,max_acc_=3.; } pp_;
Eigen::VectorXd allocate(const vector<Eigen::Vector3d>&tour,const Eigen::Vector3d&cur_vel){
const Eigen::Vector3d cur_vel_bound=cur_vel.cwiseMax(-pp_.max_vel_).cwiseMin(pp_.max_vel_);
'''+block+'''
return times;
}
int main(){
  const vector<vector<Eigen::Vector3d>> cases={
    {{0,0,0},{1,0,0},{1,3,0}},
    {{0,0,0},{1,0,0},{1,2,0},{1,2,4}},
    {{0,0,0},{0,1,0},{0,1,.2},{0,1,.4},{0,2,.4}}
  };
  bool pass=true;
  for(size_t i=0;i<cases.size();++i){
    auto t=allocate(cases[i],Eigen::Vector3d(.2,.1,0));
    const bool valid=t.allFinite() && (t.array()>0).all();
    cout<<"case="<<i<<" finite_positive="<<valid<<" durations="<<t.transpose()<<endl;
    pass=pass&&valid;
  }
  return pass?0:1;
}
'''
with tempfile.TemporaryDirectory(prefix='segment_times_') as d:
    folder=Path(d);cpp=folder/'check.cpp';cpp.write_text(program)
    build=subprocess.run(['g++','-std=c++14','-O0','-DEIGEN_INITIALIZE_MATRICES_BY_NAN',
                          '-I/usr/include/eigen3',str(cpp),'-o',str(folder/'check')],
                         capture_output=True,text=True,check=True)
    run=subprocess.run([str(folder/'check')],capture_output=True,text=True)
report={'source':str(a.source),'finite_positive_pass':run.returncode==0,
        'stdout':run.stdout,'method':'Compile actual allocation block with Eigen NaN initialization; three nonuniform multi-segment paths.',
        'limitation':'Exposes undefined input dependency, not historical performance attribution or complete trajectory feasibility.'}
if a.report:a.report.write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
raise SystemExit(run.returncode)
