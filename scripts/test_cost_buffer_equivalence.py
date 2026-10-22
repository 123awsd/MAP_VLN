"""Compile both actual production matrix functions against identical controlled costs.
Not a map replay; verifies ordering and arithmetic and allocations overhead only.
"""
from pathlib import Path
import subprocess,json
root=Path(__file__).resolve().parents[1];base=root/'runtime/consistency_validation_20261015';out=root/'outputs/consistency_validation_20261015';parts=[]
for ns,path in [('original',base/'shared_dependencies/hierarchical_grid.cpp'),('optimized',base/'A3_cost_buffers/hierarchical_grid.cpp')]:
 s=path.read_text();a=s.index('void UniformGrid::calculateCostMatrixSingleThread(');b=s.index('void UniformGrid::calculateCostMatrixMultiThread(',a);parts.append('namespace '+ns+' {\n'+r'''
class UniformGrid {
public:
 std::vector<GridCell> uniform_grid_; Config config_; Graph *connectivity_graph_;
 double getAStarCostYaw(const Position&a,const Position&b,const Eigen::Vector3d&v,double,double){++queries;for(int k=0;k<3;++k){calls.push_back(a[k]);calls.push_back(b[k]);calls.push_back(v[k]);}signature+=a.x()+3*b.y()+7*v.norm();return (int(std::abs(a.x()+b.y())*10)%7==0)?1000.:(a-b).norm()+.01;}
 unsigned long long queries=0;double signature=0;vector<double>calls;
 void calculateCostMatrixSingleThread(const Position&,const Eigen::Vector3d&,Eigen::MatrixXd&,std::map<int,std::pair<int,int>>&);
};
'''+s[a:b]+'\n}\n')
cpp=r'''
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <iostream>
#include <chrono>
#include <cassert>
#include <random>
using namespace std;using Position=Eigen::Vector3d;
struct Sink {template<class T>Sink&operator<<(const T&){return *this;}};
#define CHECK(a) if(!(a)) abort(); else Sink()
#define CHECK_GE(a,b) if(!((a)>=(b))) abort(); else Sink()
#define CHECK_GT(a,b) if(!((a)>(b))) abort(); else Sink()
struct TicToc{};
struct GridCell {int id_; vector<Position> centers_free_active_,centers_unknown_active_,centers_free_;vector<int>centers_free_active_idx_,centers_unknown_active_idx_;};
struct Config {double hybrid_search_radius_=3,unknown_penalty_factor_=1.8;};
struct Graph {unsigned long long queries=0;double signature=0;vector<pair<int,int>>calls;double searchConnectivityGraphBFS(int a,int b,vector<int>&path){++queries;calls.emplace_back(a,b);signature+=a*3+b;path={a,b};return .1+std::abs(a-b)*.04;}};
'''+''.join(parts)+r'''
int main(){std::mt19937 rng(337);double old_s=0,new_s=0;int tests=0;
for(int trial=0;trial<45;++trial){original::UniformGrid old;optimized::UniformGrid opt;Graph ga,gb;old.connectivity_graph_=&ga;opt.connectivity_graph_=&gb;
 for(int c=0;c<trial*2;++c){GridCell cell;cell.id_=c;int nf=rng()%4,nu=rng()%3;
  for(int i=0;i<nf;++i){Position p(c*.31,i*.42,(c%3)*2.8);cell.centers_free_active_.push_back(p);cell.centers_free_.push_back(p);cell.centers_free_active_idx_.push_back((c+i)%13==0?-1:i);}
  for(int i=0;i<nu;++i){cell.centers_unknown_active_.push_back(Position(c*.29,i*.37+1,(c%3)*2.8));cell.centers_unknown_active_idx_.push_back(i);}
  old.uniform_grid_.push_back(cell);
 } opt.uniform_grid_=old.uniform_grid_;Position pos(.2,1,1),vel(.3,.2,0);Eigen::MatrixXd a,b;map<int,pair<int,int>> ma,mb;
 auto t=chrono::steady_clock::now();old.calculateCostMatrixSingleThread(pos,vel,a,ma);old_s+=chrono::duration<double>(chrono::steady_clock::now()-t).count();t=chrono::steady_clock::now();opt.calculateCostMatrixSingleThread(pos,vel,b,mb);new_s+=chrono::duration<double>(chrono::steady_clock::now()-t).count();
 assert(a.rows()==b.rows()&&a.cols()==b.cols());assert((a.array()==b.array()).all());assert(ma==mb);assert(old.queries==opt.queries&&old.signature==opt.signature&&old.calls==opt.calls);assert(ga.queries==gb.queries&&ga.signature==gb.signature&&ga.calls==gb.calls);++tests;
}cout<<"{\"passed\":true,\"cases\":"<<tests<<",\"original_s\":"<<old_s<<",\"optimized_s\":"<<new_s<<",\"identical_matrix_and_center_indices\":true,\"identical_path_call_count_and_exact_order\":true}"<<endl;}
'''
p=Path('/tmp/cost_buffer_equivalence.cpp');p.write_text(cpp);subprocess.run(['g++','-std=c++14','-O2','-I/usr/include/eigen3',str(p),'-o','/tmp/cost_buffer_equivalence'],check=True);r=subprocess.run(['/tmp/cost_buffer_equivalence'],capture_output=True,text=True,check=True);result=json.loads(r.stdout.splitlines()[-1]);result['scope']='Both production matrix function bodies; controlled synthetic costs and graph; not online-map or real search timing benchmark.';(out/'cost_buffer_equivalence.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
