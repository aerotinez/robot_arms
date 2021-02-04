classdef motion_planner
    properties
        start_position;
        end_position;
        time_final;
        desired_speed;
        sample_time;
    end
    
    methods
        function obj = motion_planner(start_position, end_position, desired_speed, sample_time)
            obj.start_position = start_position;
            obj.end_position = end_position;
            obj.desired_speed = (pi/30)*desired_speed;
            obj.time_final = abs(obj.end_position - obj.start_position)/obj.desired_speed;
            obj.sample_time = sample_time;
        end
        
        function [q, qdot, qddot] = generate_trajectory(obj, current_time)
            qi = obj.start_position;
            qf = obj.end_position;
            tf = obj.time_final;
            t = current_time;
                
            A = [t^5, t^4, t^3;
                 5*t^4, 4*t^3, 3*t^2;
                 20*t^3, 12*t^2, 6*t];

            x = [6*((qf - qi)/(tf^5));
                 -15*((qf - qi)/(tf^4));
                 10*((qf - qi)/(tf^3))];

            b = A*x;

            q = b(1,1);
            qdot = b(2,1);
            qddot = b(3,1);
        end
        
        function [q, qdot, qddot] = get_trajectory(obj)
            tf = obj.time_final;
            ts = obj.sample_time;
            time = transpose(0:ts:tf);
            n = length(time);
            q = zeros(n,1);
            qdot = zeros(n,1);
            qddot = zeros(n,1);
            
            for i = 1:1:n
                t = time(i,1);
                [q(i,1), qdot(i,1), qddot(i,1)] = obj.generate_trajectory(t);
            end
        end
        
        function fig = plot_trajectory(obj, q, qdot, qddot)
            tf = obj.time_final;
            ts = obj.sample_time;
            time = 0:ts:tf;
            myBlue = (1/255)*[31, 154, 255];
            myRed = (1/255)*[255, 28, 62];
            myOrange = (1/255)*[255, 132, 31];
            
            fig = figure;
            set(gcf, 'Position', [300, 100, 600, 525])
            subplot(3,1,1);
            plot(time, q, 'Linewidth', 1.5, 'Color', myBlue);
            xlim([0, tf]);
            grid on;
            title('generated trajectory vs time');
            ylabel('position (rad)');
            subplot(3,1,2);
            plot(time, qdot, 'Linewidth', 1.5, 'Color', myRed);
            xlim([0, tf]);
            grid on;
            ylabel('velocity (rad/s)');
            subplot(3,1,3);
            plot(time, qddot, 'Linewidth', 1.5, 'Color', myOrange);
            xlim([0, tf]);
            grid on;
            ylabel('acceleration (rad/s^2)');
            xlabel('time (seconds)');
        end
    end
end