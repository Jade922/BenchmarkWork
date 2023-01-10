# A benchmark study for real-time optimal control of connected HEVs

Developing advanced control for connected hybrid electric vehicles (HEVs) has become anticipated to enhance fuel economy and mobility, while there exists a gap between the abundance of controller developments and the scarcity of bench-marking baselines. Hence, we present a benchmark study for real-time optimal control of connected HEVs, which was honored with the Gold Award for the benchmark challenge session of the IFAC Conference on Engine and Powertrain Control, Simulation and Modeling (E-CoSM) 2021. 

In this repository, we present the awarded benchmark study. The developed benchmark controller formulates a two-layer hierarchical predictive control framework, integrating eco-velocity planning in the upper layer and a power management strategy (PMS) in the lower layer, to minimize fuel consumption while enforcing constraints and satisfying multiple traffic rules. 

The E-CoSM 2021 benchmark challenge is still an open topic, the relevant benchmark problem description and the simulator can be provided via the
link: http://shenlab.jp/ecosm2021/program/benchmark-challenge.html. However, in this repository, the simulator used for testing the challengers' controllers in E-CoSM 2021 is not directly available for copyright reasons. 

Finally, we welcome the opportunity for focused discussions of the benchmark controller among relevant researchers. 

## note 
1. Initialization files
    - <font color=Red>"Initial_main.m"</font>：main function;
    - <font color=Red>"Initial_Upper.m"</font>：initialition function for upper-layer (eco-driving) controller;
    - <font color=Red>"Initial_Lower.m"</font>：initialition function for lower-layer (PMS) controller function;
