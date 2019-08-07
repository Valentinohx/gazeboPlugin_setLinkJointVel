# gazeboPlugin_setLinkJointVel
a gazebo plugin that can be used to set robot joint and robot link, this useful when testing algorithms without controller

To use:

1. modifie the joint name 
      ```ruby 
        this->model->GetJoint("KUKA_LWR4P::fkuka_lwr4p_a1_joint")->SetParam("fmax", 0, 1000.0); 
        this->model->GetJoint("KUKA_LWR4P::fkuka_lwr4p_a2_joint")->SetParam("fmax", 0, 1000.0);
       ```
        
or the link names 
        ```ruby 
        this->model->GetLink("KUKA_LWR4P::fkuka_lwr4p_A6")->SetLinearVel({_vel[0], _vel[1], _vel[2]});
        this->model->GetLink("KUKA_LWR4P::fkuka_lwr4p_A6")->SetAngularVel({_vel[3], _vel[4], _vel[5]}); 
        ```
        
based on the model used!

2. build the code 

3. attach the plugin in your urdf/sdf model
       
