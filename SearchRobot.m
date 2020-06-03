classdef SearchRobot < handle
    %SearchRobot A mobile robot with an object detector and pure pursuit
    %controller used for search robot simulation
    %   Handle so that the pose can be updated within simulation functions
    
    properties
        Detector
        MobileRobot
        Controller
        pose
    end
    
    methods
        function obj = SearchRobot()
            %SearchRobot Construct an instance of this class
            %   Initialise all the properties
            
            % Object Detector
            obj.Detector = ObjectDetector;
            obj.Detector.fieldOfView = pi/4;  % [rad]
            obj.Detector.maxRange = 5;  % [m]
            
            % Differential Drive Mobile Robot
            wheel_radius = 0.1;  % [m]
            wheel_base = 0.5;  % [m]
            obj.MobileRobot = DifferentialDrive(wheel_radius,wheel_base);
            
            % Pure Pursuit Controller
            obj.Controller = controllerPurePursuit;
            obj.Controller.LookaheadDistance = 0.13;  % [m]
            obj.Controller.DesiredLinearVelocity = 1.4;  % [m/s]
            obj.Controller.MaxAngularVelocity = 60.0;  % [rad/s]
            
            % Pose
            obj.pose = [0, 0, 0];  % [x,y,theta] 
            
        end
    end
end

