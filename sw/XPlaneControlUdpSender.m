classdef XPlaneControlUdpSender < matlab.System
    
    properties(Access = private)
        IP = "127.0.0.1"
        Port = 49000
        sampleTime = 0.02
        udpObj
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            obj.udpObj = udpport("byte", ...
                "IPV4", ...
                "Timeout", 1);
        end


        function stepImpl(obj,u)
            aileron  = u(1);
            elevator = u(2);
            rudder   = u(3);
            throttle = u(4);


            % Send control surface commands

            obj.sendDREF(...
                "sim/joystick/yoke_roll_ratio", ...
                aileron);

            obj.sendDREF(...
                "sim/joystick/yoke_pitch_ratio", ...
                elevator);

            obj.sendDREF(...
                "sim/joystick/yoke_heading_ratio", ...
                rudder);


            obj.sendDREF(...
                "sim/cockpit2/engine/actuators/throttle_ratio", ...
                throttle);

        end


        function sendDREF(obj,name,value)

            % X-Plane DREF packet
            %
            % Header:
            % "DREF0"
            %
            % float value
            %
            % dataref string (400 bytes)


            packet = zeros(413,1,'uint8');


            % Header
            packet(1:5)=uint8('DREF0');


            % Float value
            packet(6:9)=typecast(single(value),'uint8');


            % Dataref name
            nameBytes = uint8(char(name));

            packet(10:9+length(nameBytes)) = nameBytes;


            % Send packet
            write(obj.udpObj,...
                packet,...
                "uint8",...
                obj.IP,...
                obj.Port);
        end


        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,...
                "Type","Discrete",...
                "SampleTime",obj.sampleTime);
        end


        function releaseImpl(obj)
            if ~isempty(obj.udpObj)
                clear obj.udpObj
            end
        end

    end
end