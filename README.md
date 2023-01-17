# A benchmark study for real-time optimal control of connected HEVs

Developing advanced control for connected hybrid electric vehicles (HEVs) has become anticipated to enhance fuel economy and mobility, while there exists a gap between the abundance of controller developments and the scarcity of bench-marking baselines. Hence, we present a benchmark study for real-time optimal control of connected HEVs, which was honored with the Gold Award for the benchmark challenge session of the IFAC Conference on Engine and Powertrain Control, Simulation and Modeling (E-CoSM) 2021. 

In this repository, we present the awarded benchmark study. The developed benchmark controller formulates a two-layer hierarchical predictive control framework, integrating eco-velocity planning in the upper layer and a power management strategy (PMS) in the lower layer, to minimize fuel consumption while enforcing constraints and satisfying multiple traffic rules. 

The E-CoSM 2021 benchmark challenge is still an open topic, the relevant benchmark problem description and the simulator can be provided via the
link: http://shenlab.jp/ecosm2021/program/benchmark-challenge.html. However, in this repository, the simulator used for testing the challengers' controllers in E-CoSM 2021 is not directly available for copyright reasons. 

Finally, we welcome the opportunity for focused discussions of the benchmark controller among relevant researchers. 

## note 
1. Initialization files
    - <font color=Red>**"Initial_main.m"**</font>：main function;
    - <font color=Red>**"Initial_Upper.m"**</font>：initialition function for upper-layer (eco-driving) controller;
    - <font color=Red>**"Initial_Lower.m"**</font>：initialition function for lower-layer (PMS) controller function;
2. Simulator (**"Benchmark2021Controller.slx"**)
    
    This simulator presents the benchmark controller, a two-layer hierarchical control architecture, which incorporates different scales of the connected information. In the upper layer, an MPC-based controller (**"UpperLayerVPlan"**) is designed to plan eco-trajectory and handle traffic rules, with the informed knowledge of the geographical information (e.g., road slope), the traffic lights information, and the future velocity of the preceding vehicle estimated by a GP-based predictor (**"UpperLayerVpredict"**). Then, the planned velocity trajectory is passed on to the driver model (**"DriverModel"**) to determine the reference of traction power. In the lower layer, a real-time power management controller (**"LowerLaverPMP"**) is designed to minimize the fuel consumption where the optimal power split between mechanical engine power and battery electrical power is determined. Details of the controller are elaborated in the model-*Benchmark2021Controller.slx*. 
