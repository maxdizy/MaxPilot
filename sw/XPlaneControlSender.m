classdef XPlaneControlSender < matlab.System
    % Sends aircraft control commands to X-Plane 12
    % Input:
    % u(1) = aileron  (-1 to 1)
    % u(2) = elevator (-1 to 1)
    % u(3) = rudder   (-1 to 1)
    % u(4) = throttle (0 to 1)

    properties (Access = private)
        URL
        sampleTime = 0.02;

        % dataref IDs
        aileronID
        elevatorID
        rudderID
        throttleID

        overrideControlID
        overrideJoystickID
        
        % control surface max and min deflections (deg)
        % Cirrus SR22
        aileronMax = 10;
        aileronMin = -10; % actually -30 but keeping symmetry
        elevatorMax = 20;
        elevatorMin = -25;
        rudderMax = 15;
        rudderMin = -15;

        % actuator states
        aileronState = 0;
        elevatorState = 0;
        rudderState = 0;

        % actuator rate limit (deg/s)
        actuatorRateLimit = 600;
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.URL = "http://127.0.0.1:8086/api/v3";

            obj.aileronID = obj.getID("sim/flightmodel2/wing/aileron1_deg");
            obj.elevatorID = obj.getID("sim/flightmodel2/wing/elevator1_deg");
            obj.rudderID = obj.getID("sim/flightmodel2/wing/rudder1_deg");
            obj.throttleID = obj.getID("sim/cockpit2/engine/actuators/throttle_ratio_all");

            obj.overrideControlID = obj.getID("sim/operation/override/override_control_surfaces");
            obj.overrideJoystickID = obj.getID("sim/operation/override/override_joystick");

            obj.setDREF(obj.overrideControlID, 1, false, false, false);
            obj.setDREF(obj.overrideJoystickID, 1, false, false, false);
        end

        function stepImpl(obj, u)
            aileronCmd = max(min(u(1),obj.aileronMax),obj.aileronMin);
            elevatorCmd = max(min(u(2),obj.elevatorMax),obj.elevatorMin) * -1; % flip direction
            rudderCmd = max(min(u(3),obj.rudderMax),obj.rudderMin);
            throttleCmd = max(min(u(4),1),0);

            % send commands -- indices need to be adjusted per aircraft
            % Cirrus SR-22
            obj.setDREF(obj.aileronID, aileronCmd, true, false, false, 4);
            obj.setDREF(obj.elevatorID, elevatorCmd, false, true, false, 8);
            obj.setDREF(obj.rudderID, rudderCmd, false, false, true, 10);
            obj.setDREF(obj.throttleID, throttleCmd, false, false, false);

            % % temp send overrides
            % obj.setDREF(obj.overrideControlID, 0, false, false, false);
            % obj.setDREF(obj.overrideJoystickID, 0, false, false, false);
        end
    end

    methods (Access = private)
        % get dataref id from X-Plane REST API
        function id = getID(obj, name)
            coder.extrinsic('webread');
            result = webread(obj.URL + "/datarefs", "filter[name]", name);
            id = 0;
            if isstruct(result)
                id = result.data.id;
            end
        end

        % send commands to X-Plane REST API
        function setDREF(obj, id, value, isAileron, isElevator, isRudder, index)
            coder.extrinsic('webread');
            coder.extrinsic('webwrite');
            coder.extrinsic('weboptions');

            body = struct();
            if isRudder
                body.data = zeros(48,1);
                delta = value - obj.rudderState;
                delta = max(min(delta, obj.actuatorRateLimit * obj.sampleTime), -(obj.actuatorRateLimit * obj.sampleTime));
                obj.rudderState = obj.rudderState + delta;
                body.data(index+1) = obj.rudderState;
            elseif isElevator
                body.data = zeros(48,1);
                delta = value - obj.elevatorState;
                delta = max(min(delta, obj.actuatorRateLimit * obj.sampleTime), -(obj.actuatorRateLimit * obj.sampleTime));
                obj.elevatorState = obj.elevatorState + delta;
                body.data(index+1) = obj.elevatorState;
            elseif isAileron
                body.data = zeros(48,1);
                delta = value - obj.aileronState;
                delta = max(min(delta, obj.actuatorRateLimit * obj.sampleTime), -(obj.actuatorRateLimit * obj.sampleTime));
                obj.aileronState = obj.aileronState + delta;
                body.data(index+1) = obj.aileronState;
                body.data(index+2) = -obj.aileronState;
            else
                body.data = value;
            end

            options = weboptions('RequestMethod', 'patch', ...
                                'MediaType', 'application/json');

            webwrite(obj.URL + "/datarefs/" + string(id) + "/value", body, options);
        end
    end

    % structure input data
    methods (Access = protected)
        function in = getInputSizeImpl(~)
            in = [4 1];
        end

        function in = getInputDataTypeImpl(~)
            in = "double";
        end

        function in = isInputComplexImpl(~)
            in = false;
        end

        function in = isInputFixedSizeImpl(~)
            in = true;
        end

        % function sts = getSampleTimeImpl(obj)
        %     sts = createSampleTime(obj, "Type", "Discrete", "SampleTime", obj.sampleTime);
        % end
    end
end