classdef pointmass
    properties
        mass
        position
        velocity
        orientation
        translation_force_sum
    end
    methods
        function [position, velocity] = solve(obj,dt)
            a = obj.translation_force_sum/obj.mass;
            velocity = obj.velocity + a*dt;
            position = obj.position + obj.velocity*dt;
        end
    end
end