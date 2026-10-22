"""Compile the production budget class and replay measured deadlines; no map replay."""
from pathlib import Path
import subprocess,json,statistics
root=Path(__file__).resolve().parents[1];r=root/'runtime/consistency_validation_20261015';s=(r/'A3_deadline_schedule/exploration_fsm.h').read_text();begin=s.index('namespace fast_planner {\n// Samples');end=s.index('\nnamespace fast_planner {',begin+1);budget=s[begin:end];prefix='#include <algorithm>\n#include <deque>\n#include <vector>\n#include <cmath>\n#include <cassert>\n#include <iostream>\n';main=r'''
int main(){using fast_planner::HandoverBudget;HandoverBudget b;
assert(b.shouldPlan(.05));assert(b.shouldPlan(.7));assert(!b.shouldPlan(3));
double initial=b.lead();b.observeSolve(1.4);assert(b.lead()>initial);assert(b.shouldPlan(1.5));
assert(!b.ready(10.,9.9));assert(!b.ready(10.,10.02));assert(b.ready(10.,10.1));
b.observeDelivery(.3);assert(b.lead()>=1.78);
for(int i=0;i<40;++i)b.observeSolve(.4);assert(b.lead()<1.);
b.observeSolve(NAN);b.observeDelivery(-1.);assert(std::isfinite(b.lead()));
std::cout<<"{\"passed\":true,\"checks\":{\"triggers_before_old_50ms_threshold\":true,\"measured_solve_expands_budget\":true,\"late_completion_rejected_without_retimestamp\":true,\"delivery_budget_accounted\":true,\"finite_window_adapts_down\":true,\"invalid_samples_ignored\":true}}";
}
''';tmp=Path('/tmp/handover_budget_test.cpp');tmp.write_text(prefix+budget+main);subprocess.run(['g++','-std=c++14','-O2',str(tmp),'-o','/tmp/handover_budget_test'],check=True);result=json.loads(subprocess.check_output(['/tmp/handover_budget_test'],text=True));result['limits']=['Tests compiled production budget policy; not a physical trajectory, full map, or coverage replay.'];out=root/'outputs/consistency_validation_20261015/handover_budget_test.json';out.write_text(json.dumps(result,indent=2));print(json.dumps(result))
