With development of onboard sensor and onboard computer, UAVs gain capability for complex tasks and have been used broadly in various fields. For motion planning of UAVs, movement constraints such as kinematics and dynamics are taken into account to obtain optimal trajectories, which benefit for efficient and safe flight. RGB-D cameras are widely utilized as onboard sensors due to traits of informative, high precision, and low cost, but their confined sensing range brings constraints for UAV motion planning.

In this project, an autonomous navigation strategy is proposed for unmanned aerial vehicles based on consideration of dynamic sampling and field of view. Compare to search-based motion planning, sampling-based kinodynamic planning schemes can often find feasible trajectories in complex environments. Specifically, a global trajectory is first generated with physical information, and an expansion algorithm is constructed regarding to kinodynamic rapidly-exploring random tree* (KRRT*). 

Then, a KRRT* expansion strategy is designed to find local collision-free trajectories, In trajectory optimization, bending radius, collision risk function, and yaw angle penalty term are defined by taking into account onboard sensor FOV and potential risk. Effectiveness of the proposed strategy is demonstrated by both simulation and experiment.

For more technical details, please refer to:

H. Yin, B. Li, H. Zhu, and L. Shi, Kinodynamic RRT* based UAV optimal state motion planning with collision risk awareness. Information Technology and Control, vol. 52, no. 3, pp. 665-678, 2023. 
