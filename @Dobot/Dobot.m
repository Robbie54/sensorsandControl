classdef Dobot < RobotBaseClass
    

    properties(Access = public)   
        plyFileNameStem = '';
        defaultBaseTr = eye(4);
%         initalQ = [0, 0, -pi/2, 0, -pi/2, 0 ,0];
        %defaultWorkspace = [-2 2 -2 2 -2 2];
        defaultWorkspace = [-0.5 0.5 -0.5 0.5 -0.5 0.5];
        
    end
    
    methods
%% Constructor
function self = Dobot(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                   % baseTr = [1,0,0,0.4; 0,1,0,0; 0,0,1,1; 0,0,0,1];  
                    baseTr = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1]; %change this to move dobot starting location
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.model.tool = self.toolTr;
%             self.homeQ = self.initalQ;
            self.workspace = self.defaultWorkspace;
            self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)

            %refer to notes on dobot arm pdf
            link(1) = Link('d',0.103,'a',0,'alpha',-pi/2,'offset',0,'qlim', deg2rad([-135,135]));
            link(2) = Link('d',0,'a',0.136,'alpha',0,'offset',-pi/2,'qlim', deg2rad([-5, 85]));
            link(3) = Link('d',0,'a',0.1685,'alpha',0,'offset',0,'qlim', deg2rad([-10,95]));
            link(4) = Link('d',0,'a',0.0525,'alpha',-pi/2,'offset',0,'qlim', deg2rad([-90,90]));
            link(5) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', deg2rad([-85,85]));
           
             
            self.model = SerialLink(link,'name',self.name);
        end   
        
   
 %% Plot and Colour Dobot
     function PlotAndColourRobot(self)

            for linkIndex = 0:self.model.n
                [ faceData, vertexData, data{linkIndex + 1} ] = plyread(['Link' ...
                    ,num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'wrist', 'arrow','workspace' ...
                ,self.workspace, 'delay', 0);
            axis([self.workspace(1) self.workspace(2) self.workspace(3) self.workspace(4) ...
                self.workspace(5) self.workspace(6)]);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [data{linkIndex+1}.vertex.red ...
                        , data{linkIndex+1}.vertex.green ...
                        , data{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
    end
end