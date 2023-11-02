classdef eStop < handle
    properties
        EStopPressed = false;
        option = 0;
        GUIestop = []; % Store the GUI handle
    end

    events
        ContinuePressed
    end

    methods
        function self = eStop()
            % Constructor (if needed)
            % Initialize properties or perform setup here
        end

        function press(self)
            if self.EStopPressed
                if isempty(self.GUIestop)
                    self.GUIestop = eStopGUI();
                    drawnow();
                end

                % Set up a listener to watch for the "Continue" button press
                setappdata(self.GUIestop, 'EStopListener', addlistener(self.GUIestop, 'ContinuePressed', @(src, event) self.onContinuePressed()));

                % Don't delete the GUI here, let it stay open
                % until the user selects an option in the GUI
            end
        end

        function onContinuePressed(self)
            if ~isempty(self.GUIestop)
                selectedOption = self.GUIestop.getSelectedOption(); % Get the user's selection from the GUI

                if selectedOption == 1
                    fprintf('\nContinue selected');
                    self.option = 1;
                end

                delete(self.GUIestop);
                self.GUIestop = []; % Reset the GUI handle
            end
        end
   end
end
