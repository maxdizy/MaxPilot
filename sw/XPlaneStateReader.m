classdef XPlaneStateReader < matlab.System
    % Reads X-Plane 12 aircraft state using the Web API
    % Output: [u v w p q r phi theta psi North East alt]
    % 1  - u      body velocity X (m/s)
    % 2  - v      body velocity Y (m/s)
    % 3  - w      body velocity Z (m/s)
    % 4  - p      roll rate (rad/s)
    % 5  - q      pitch rate (rad/s)
    % 6  - r      yaw rate (rad/s)
    % 7  - phi    roll angle (rad)
    % 8  - theta  pitch angle (rad)
    % 9  - psi    yaw angle (rad)
    % 10 - North  local North position (m)
    % 11 - East   local East position (m)
    % 12 - elev    elevation (m)

    properties (Access = private)
        URL
        sampleTime = 0.02;

        lat0
        lon0
        elev0
        
        latID
        lonID
        elevID
        phiID
        thetaID
        psiID
        pID
        qID
        rID
        uID
        vID
        wID

        R = 6378137;
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.URL = "http://127.0.0.1:8086/api/v3";

            obj.latID = obj.getID("sim/flightmodel/position/latitude");
            obj.lonID = obj.getID("sim/flightmodel/position/longitude");
            obj.elevID = obj.getID("sim/flightmodel/position/elevation");

            obj.phiID = obj.getID("sim/flightmodel/position/phi");
            obj.thetaID = obj.getID("sim/flightmodel/position/theta");
            obj.psiID = obj.getID("sim/flightmodel/position/psi");

            obj.pID = obj.getID("sim/flightmodel/position/P");
            obj.qID = obj.getID("sim/flightmodel/position/Q");
            obj.rID = obj.getID("sim/flightmodel/position/R");

            obj.uID = obj.getID("sim/flightmodel/position/local_vx");
            obj.vID = obj.getID("sim/flightmodel/position/local_vy");
            obj.wID = obj.getID("sim/flightmodel/position/local_vz");

            obj.lat0 = obj.getDREF(obj.latID);
            obj.lon0 = obj.getDREF(obj.lonID);
            obj.elev0 = obj.getDREF(obj.elevID);
        end

        function state = stepImpl(obj)
            lat = obj.getDREF(obj.latID);
            lon = obj.getDREF(obj.lonID);
            elev = obj.getDREF(obj.elevID);
            % alt = obj.getDREF(obj.elevID) - obj.elev0;

            lat0_rad = deg2rad(obj.lat0);
            lon0_rad = deg2rad(obj.lon0);
            lat_rad = deg2rad(lat);
            lon_rad = deg2rad(lon);

            North = obj.R*(lat_rad-lat0_rad);
            East = obj.R*cos(lat0_rad)*(lon_rad-lon0_rad);

            phi = obj.getDREF(obj.phiID);
            theta = obj.getDREF(obj.thetaID);
            psi = obj.getDREF(obj.psiID);

            p = obj.getDREF(obj.pID);
            q = obj.getDREF(obj.qID);
            r = obj.getDREF(obj.rID);

            u = obj.getDREF(obj.uID);
            v = obj.getDREF(obj.vID);
            w = obj.getDREF(obj.wID);

            state = [
                u;
                v;
                w;
                p;
                q;
                r;
                phi;
                theta;
                psi;
                North;
                East;
                elev
            ];
        end

        function resetImpl(obj)
            obj.lat0 = 0;
            obj.lon0 = 0;
            obj.elev0 = 0;
        end

    end

    methods (Access = private)
        function id = getID(obj, name)
            coder.extrinsic('webread');
            result = webread(obj.URL + "/datarefs", "filter[name]", name);
            id = 0;
            if isstruct(result)
                id = result.data.id;
            end
        end

        function value = getDREF(obj, id)
            coder.extrinsic('webread');
            result = webread(obj.URL + "/datarefs/" + string(id) + "/value");
            value = 0;
            if isstruct(result)
                value = result.data;
            end
        end
    end

    methods (Access = protected)
        function out = getOutputSizeImpl(~)
            out = [12 1];
        end

        function out = getOutputDataTypeImpl(~)
            out = "double";
        end

        function out = isOutputComplexImpl(~)
            out = false;
        end

        function out = isOutputFixedSizeImpl(~)
            out = true;
        end
        
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, "Type", "Discrete", "SampleTime", obj.sampleTime);
        end
    end
end