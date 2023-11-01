% function UR3Control(app,robot)
% %     clc;
% %     clf;
% %     clear all; 
% % 
% %     r1 = UR3();
% %     r1.model.teach;
% %     ControlGui = UR3GUI();
% %     while(1)
% %         pause(0.001);
% %         ControlGui.ControlGuiUpdate(r1);
% %         if ControlGui.Exit == true
% %             fprintf('\nExited Control Mode')
% %             delete(ControlGui);
% %             break;
% %         end
% %     end
% % end
% % function ControlGuiUpdate(app,robot)
%             app.UR3_P = [app.q1     app.q2     app.q3     app.q4     app.q5     app.q6];
%             app.UR3_T = robot.model.fkineUTS(app.UR3_P);
%             rpy = rad2deg(tr2rpy(app.UR3_T));
% 
%             app.XTextArea.Value = num2str(app.UR3_T(1,4));
%             app.YTextArea.Value = num2str(app.UR3_T(2,4));
%             app.ZTextArea.Value = num2str(app.UR3_T(3,4));
%             app.RollTextArea.Value = num2str(rpy(1));
%             app.PitchTextArea.Value = num2str(rpy(2));
%             app.YawTextArea.Value = num2str(rpy(3));
% 
%             app.TextArea2.Value = num2str(rad2deg(app.UR3_P(1)));
%             app.TextArea2_2.Value = num2str(rad2deg(app.UR3_P(2)));
%             app.TextArea2_4.Value = num2str(rad2deg(app.UR3_P(3)));
%             app.TextArea2_5.Value = num2str(rad2deg(app.UR3_P(4)));
%             app.TextArea2_3.Value = num2str(rad2deg(app.UR3_P(5)));
%             app.TextArea2_6.Value = num2str(rad2deg(app.UR3_P(6)));
% 
%         end