function LL6Control()
    r2.model.teach;
    ControlGui = LinearLite6GUI();
    while(1)
        pause(0.001);
        ControlGui.ControlGuiUpdate(r2);
        if ControlGui.Exit == true
            fprintf('\nExited Control Mode')
            delete(ControlGui);
            break;
        end
    end
end