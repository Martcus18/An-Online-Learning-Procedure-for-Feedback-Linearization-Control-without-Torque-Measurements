%Kuka LWR class




classdef lwr
    properties
        robot
    end
  
    methods
        function obj = lwr(inertia_path,masses_path)
            
            dhparams = load("DH.txt");
            
            masses = load(masses_path);
            CoMs = load("center_of_masses.txt");

            Inertia = load(inertia_path);
            
            Inertia(1,:) = [Inertia(1,1),Inertia(1,4),Inertia(1,6),Inertia(1,5),Inertia(1,3),Inertia(1,2)];
            Inertia(2,:) = [Inertia(2,1),Inertia(2,4),Inertia(2,6),Inertia(2,5),Inertia(2,3),Inertia(2,2)];
            Inertia(3,:) = [Inertia(3,1),Inertia(3,4),Inertia(3,6),Inertia(3,5),Inertia(3,3),Inertia(3,2)];
            Inertia(4,:) = [Inertia(4,1),Inertia(4,4),Inertia(4,6),Inertia(4,5),Inertia(4,3),Inertia(4,2)];
            Inertia(5,:) = [Inertia(5,1),Inertia(5,4),Inertia(5,6),Inertia(5,5),Inertia(5,3),Inertia(5,2)];
            Inertia(6,:) = [Inertia(6,1),Inertia(6,4),Inertia(6,6),Inertia(6,5),Inertia(6,3),Inertia(6,2)];
            Inertia(7,:) = [Inertia(7,1),Inertia(7,4),Inertia(7,6),Inertia(7,5),Inertia(7,3),Inertia(7,2)];
            
            robot = robotics.RigidBodyTree('DataFormat','row');
            
            %Set body and joints
            
            body1 = robotics.RigidBody('body1');
            jnt1 = robotics.Joint('jnt1','revolute');
            
            body2 = robotics.RigidBody('body2');            
            jnt2 = robotics.Joint('jnt2','revolute');
            
            body3 = robotics.RigidBody('body3');            
            jnt3 = robotics.Joint('jnt3','revolute');
            
            body4 = robotics.RigidBody('body4');            
            jnt4 = robotics.Joint('jnt4','revolute');
 
            body5 = robotics.RigidBody('body5');            
            jnt5 = robotics.Joint('jnt5','revolute');
            
            body6 = robotics.RigidBody('body6');            
            jnt6 = robotics.Joint('jnt6','revolute');
            
            body7 = robotics.RigidBody('body7');            
            jnt7 = robotics.Joint('jnt7','revolute');
            
            setFixedTransform(jnt1,dhparams(1,:),'dh');
            setFixedTransform(jnt2,dhparams(2,:),'dh');
            setFixedTransform(jnt3,dhparams(3,:),'dh');
            setFixedTransform(jnt4,dhparams(4,:),'dh');
            setFixedTransform(jnt5,dhparams(5,:),'dh');
            setFixedTransform(jnt6,dhparams(6,:),'dh');
            setFixedTransform(jnt7,dhparams(7,:),'dh');
            
            body1.Joint = jnt1;
            body2.Joint = jnt2;
            body3.Joint = jnt3;
            body4.Joint = jnt4;
            body5.Joint = jnt5;
            body6.Joint = jnt6;
            body7.Joint = jnt7;
            
            
%            Set dynamics parameters
            
            body1.Mass = masses(1);
            body2.Mass = masses(2);
            body3.Mass = masses(3);
            body4.Mass = masses(4);
            body5.Mass = masses(5);
            body6.Mass = masses(6);
            body7.Mass = masses(7);
            
            body1.CenterOfMass = CoMs(1,:);
            body2.CenterOfMass = CoMs(2,:);
            body3.CenterOfMass = CoMs(3,:);
            body4.CenterOfMass = CoMs(4,:);
            body5.CenterOfMass = CoMs(5,:);
            body6.CenterOfMass = CoMs(6,:);
            body7.CenterOfMass = CoMs(7,:);
            
            body1.Inertia = Inertia(1,:);
            body2.Inertia = Inertia(2,:);
            body3.Inertia = Inertia(3,:);
            body4.Inertia = Inertia(4,:);
            body5.Inertia = Inertia(5,:);
            body6.Inertia = Inertia(6,:);
            body7.Inertia = Inertia(7,:);
            
            addBody(robot,body1,'base');
            addBody(robot,body2,'body1');
            addBody(robot,body3,'body2');
            addBody(robot,body4,'body3');
            addBody(robot,body5,'body4');
            addBody(robot,body6,'body5');
            addBody(robot,body7,'body6');
                        
            robot.Gravity = [0 0 -9.81];
            obj.robot = robot;
        end
    end      
end
