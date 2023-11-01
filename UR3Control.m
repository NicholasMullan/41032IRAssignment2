function UR3Control()
    r1 = UR3();
    r1.model.teach;
    ControlGui = UR3GUI();
    while(1)
        pause(0.001);
        ControlGui.ControlGuiUpdate(r1);
        if ControlGui.Exit == true
            fprintf('\nExited Control Mode')
            delete(ControlGui);
            break;
        end
    end
end