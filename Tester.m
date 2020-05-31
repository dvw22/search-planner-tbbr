classdef Tester < handle
    %UNTITLED7 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        test1
        test2
    end
    
    methods
        function obj = Tester()
            %UNTITLED7 Construct an instance of this class
            %   Detailed explanation goes here
            obj.test1 = 1;
            obj.test2 = 2;
        end

%         function obj = set.test1(obj,new)
%             obj.test1 = new;
%         end
        
        function [obj] = set1_3(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.test1 = 3;
            
        end
    end
end

