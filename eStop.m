% classdef eStop < handle
%     properties
%         isPressed = false;
%         option = 0;
%     end
% 
%     methods
%         function self = eStop()
% 
%         end
% 
% 
% 
%         function press(self)
% 
%             if self.isPressed == true
%                 GUIestop = eStopGUI();
%                 drawnow();
% 
%                 while self.isPressed == true
%                     pause(0.001);
%                     if self.isPressed == GUIestop.Continue
%                         fprintf('\nContinue selected')
%                         self.option = 1;
%                         self.isPressed = false;
%                     elseif self.isPressed == GUIestop.ResetUR3
%                         fprintf('\nReset UR3 selected')
%                         self.option = 2;
%                         self.isPressed = false;
%                     elseif self.isPressed == GUIestop.ControlUR3
%                         fprintf('\nControl UR3 selected')
%                         self.option = 3;
%                         self.isPressed = false;
%                     elseif self.isPressed == GUIestop.ResetLinearLite6
%                         fprintf('\nReset LinearLite6 selected')
%                         self.option = 4;
%                         self.isPressed = false;
%                     elseif self.isPressed == GUIestop.ControlLinearLite6
%                         fprintf('\nControl LinearLite6 selected')
%                         self.option = 5;
%                         self.isPressed = false;
%                     end
% 
% 
%                 end
%                 delete(GUIestop);
%             end
% 
%         end
%     end
% end
