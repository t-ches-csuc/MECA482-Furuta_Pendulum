# MECA482-Furuta_Pendulum

<Header> <b> Introduction </b> </Header>

<body>
  <p>
    A Futura Pendulum is a simple Control Theroy learning tool. It is comprised of a rotating arm and a pendulum attached to the end. This project is a 
    review of a developed full state feed back control model created in Simulink [1]. The model neglects friction, when considering the mass relative to the fricton of       the bearings in the pendulum it is a good assumtion that the coefficient of friction is too small to significantly impact the model. 
  </p>
  
  <header> <b> Operational and Logical Viewpoints </b> </Header>
  
   <p align="center">
  <img width="750" alt="operationalViewpoint" src="Functional Viewpoint.PNG">
  
  <p align="center">
   <b>Figure 1</b>: Operational viewpoint for the inertia wheel pendulum.
  <p>
  
  <p align="center">
   <img width="600" alt="logicalViewpoint" src="Operational Viewpoint.PNG">
  <p>
  
  <p align="center">
   <b>Figure 2</b>: Logical/functional viewpoint for the inertia wheel pendulum.
  <p>
  
  <Header> <b> Mathematical Model </b> </Header>
  <p>
    The system is modeled as a 2 DOF system. One rotation about the axis of the motor and one rotation about the axis of the arm which the pendulum rotates around. A
    system was developed which was non-linear. Using state-space representation the final matricies were found to be:
    <p>
    [Super hot matrices go here]
    </p>
    Using computational solvers the poles of the system can be found to be:
    <p><br>
    [POLES graph here]
    </p>
    As shown in the graph below, the system is a stable system as modeled in this simulation. Adding in real variables such as friction may move the poles closer 
    to the right and thus closer to instability.
  </p>
  <Header> <b> MATLAB Implementation </b> </Header>
  <p> 
    The MATLAB implementation developed by Vikash Gupta uses the mathematical model stated previously and the calculated poles to produce the useful variables for the
    Simulink model. System variables are input in the first lines and used to compute the equation of motion for a particular system. Notably, the MATLAB code neglects 
    the amplitude of the simulated sinusoidal input. This input was subsequently defined and included in the system as an amplidtued of 0.1.
  </p>
  <Header> <b> Simulink Integration and Simulation </b> </Header>
  <p> 
    
  </p>
  <Header> <b> Conclusion </b> </Header>
  <p> </p>
</body>
