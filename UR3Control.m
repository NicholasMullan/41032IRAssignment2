function UR3Control()
    UR3.model.teach;
    ControlGui = UR3GUI();
    while(1)
        pause(0.001);
        ControlGui.ControlGuiUpdate(UR3);
        if ControlGui.Exit == true
            fprintf('\nExited Control Mode')
            delete(ControlGui);
            break;
        end
    end
end