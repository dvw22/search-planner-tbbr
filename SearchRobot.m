classdef SearchRobot < handle
    %SearchRobot Mobile search robot for search simulation
    %   This class is a handle because the methods are designed to update 
    %   the object's properties. 
    %
    %   The SearchRobot class initialises a mobile robot object with an
    %   object detector and pure pursuit controller, to be used during
    %   simulation of a mobile search robot. The pure pursuit controller
    %   has a very high max angular velocity and short look ahead distance
    %   to emulate the movement behaviour of a turtlebot. This results in
    %   some oscillation during sharp turns in the simulation.
    
    properties
        Detector  % object detector object on robot
        MobileRobot  % mobile robot object
        Controller  % pure pursuit controller object for motion
        pose  % starting pose of the robot
    end
    
    methods
        function obj = SearchRobot()
            %SearchRobot Construct an instance of this class
 
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

