function LL6Control()
    LinearLite6.model.teach;
    ControlGui = LinearLite6GUI();
    while(1)
        pause(0.001);
        ControlGui.ControlGuiUpdate(LinearLite6);
        if ControlGui.Exit == true
            fprintf('\nExited Control Mode')
            delete(ControlGui);
            break;
        end
    end
end