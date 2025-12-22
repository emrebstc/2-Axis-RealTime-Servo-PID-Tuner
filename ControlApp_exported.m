classdef ControlApp_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                 matlab.ui.Figure
        TemizleButton_2          matlab.ui.control.Button
        TemizleButton            matlab.ui.control.Button
        SetpointYSlider          matlab.ui.control.Slider
        SetpointYSliderLabel     matlab.ui.control.Label
        SetpointXSlider          matlab.ui.control.Slider
        SetpointXSliderLabel     matlab.ui.control.Label
        GnderButton_8            matlab.ui.control.Button
        GnderButton_7            matlab.ui.control.Button
        GnderButton_6            matlab.ui.control.Button
        GnderButton_5            matlab.ui.control.Button
        GnderButton_4            matlab.ui.control.Button
        GnderButton_3            matlab.ui.control.Button
        Kd_yEditField            matlab.ui.control.NumericEditField
        Kd_yEditFieldLabel       matlab.ui.control.Label
        Ki_yEditField            matlab.ui.control.NumericEditField
        Ki_yEditFieldLabel       matlab.ui.control.Label
        Kp_yEditField            matlab.ui.control.NumericEditField
        Kp_yEditFieldLabel       matlab.ui.control.Label
        Kd_xEditField            matlab.ui.control.NumericEditField
        Kd_xEditFieldLabel       matlab.ui.control.Label
        Ki_xEditField            matlab.ui.control.NumericEditField
        Ki_xEditFieldLabel       matlab.ui.control.Label
        Kp_xEditField            matlab.ui.control.NumericEditField
        Kp_xEditFieldLabel       matlab.ui.control.Label
        PORTDropDown             matlab.ui.control.DropDown
        PORTDropDownLabel        matlab.ui.control.Label
        GnderButton_2            matlab.ui.control.Button
        GnderButton              matlab.ui.control.Button
        SetpointYEditField       matlab.ui.control.NumericEditField
        SetpointYEditFieldLabel  matlab.ui.control.Label
        SetpointXEditField       matlab.ui.control.NumericEditField
        SetpointXEditFieldLabel  matlab.ui.control.Label
        UIAxes2                  matlab.ui.control.UIAxes
        UIAxes                   matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        com %Port
        % Grafik ve hareketli çizgiler
        LineX        % X ekseni verisi
        LineXSet     % X setpoint
        LineY        % Y ekseni verisi
        LineYSet     % Y setpoint
        StartTime    % Grafik zaman referansi
               
    end
    
    methods (Access = private)
     function readSerialData(app, src, ~)
            try
                line = readline(src);
                data = str2num(line); 
                
                if length(data) >= 4
                    % Gecen süreyi saniye cinsinden hesaplama
                    t = seconds(datetime('now') - app.StartTime);
                    
                    % X Grafigine veri ekleme
                    addpoints(app.LineX, t, data(1));
                    addpoints(app.LineXSet, t, data(2));
                    
                    % Y Grafigine veri ekleme
                    addpoints(app.LineY, t, data(3));
                    addpoints(app.LineYSet, t, data(4));
                    
                    % Eksenleri kaydirma (Son 100 saniyeyi gösterme)
                    if t > 100
                        app.UIAxes.XLim = [t-100, t];
                        app.UIAxes2.XLim = [t-100, t];
                    else
                        app.UIAxes.XLim = [0, 100];
                        app.UIAxes2.XLim = [0, 100];
                    end
                    
                    % Ekranı güncelleme sıklıgı
                    drawnow limitrate; 
                end
            catch
                % Hatali veri varsa bir sey yapma
            end
        end
    end
  

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
    ports = serialportlist("available");
    app.PORTDropDown.Items = ports;
    
       % Grafikleri Hazırlama
            app.StartTime = datetime('now');
            
            % X Ekseni Grafiği
            app.LineX = animatedline(app.UIAxes, 'Color', 'b', 'LineWidth', 1.5);
            app.LineXSet = animatedline(app.UIAxes, 'Color', 'r', 'LineStyle', '--');
            legend(app.UIAxes, {'Derece', 'Setpoint'});
            
            % Y Ekseni Grafiği
            app.LineY = animatedline(app.UIAxes2, 'Color', 'b', 'LineWidth', 1.5);
            app.LineYSet = animatedline(app.UIAxes2, 'Color', 'r', 'LineStyle', '--');
            legend(app.UIAxes2, {'Derece', 'Setpoint'});
        end

        % Value changed function: PORTDropDown
        function PORTDropDownValueChanged(app, event)
 selectedPort = app.PORTDropDown.Value;
    if ~isempty(app.com)
        delete(app.com);
    end
    
    app.com = serialport(selectedPort, 115200);
    app.com.Timeout = 0.5; % Daha kararlı bir iletiim icin
    configureTerminator(app.com, "LF");
    
    flush(app.com);
    configureCallback(app.com, "terminator", @app.readSerialData);
        end

        % Button pushed function: GnderButton
        function GnderButtonPushed(app, event)
  if ~isempty(app.com) && isvalid(app.com)
        val = app.SetpointXEditField.Value;
        app.SetpointXSlider.Value = val;
        flush(app.com, "output"); 
        % char(10) \n (LF) karakteri olarak manuel ekleme
        write(app.com, sprintf("X%.2f%s", val, char(10)), "string"); 
    end
        end

        % Button pushed function: GnderButton_2
        function GnderButton_2Pushed(app, event)
  if ~isempty(app.com) && isvalid(app.com)
        val = app.SetpointYEditField.Value;
        app.SetpointYSlider.Value = val;
        flush(app.com, "output");
        write(app.com, sprintf("Y%.2f%s", val, char(10)), "string");
    end
        end

        % Button pushed function: GnderButton_3
        function GnderButton_3Pushed(app, event)
  if ~isempty(app.com) && isvalid(app.com)
        val = app.Kp_xEditField.Value;
        flush(app.com, "output");
        write(app.com, sprintf("PX%.3f%s", val, char(10)), "string");
    end
        end

        % Button pushed function: GnderButton_4
        function GnderButton_4Pushed(app, event)
   if ~isempty(app.com) && isvalid(app.com)
        val = app.Ki_xEditField.Value;
        flush(app.com, "output");
        write(app.com, sprintf("IX%.3f%s", val, char(10)), "string");
    end
        end

        % Button pushed function: GnderButton_5
        function GnderButton_5Pushed(app, event)
if ~isempty(app.com) && isvalid(app.com)
        val = app.Kd_xEditField.Value;
        flush(app.com, "output");
        write(app.com, sprintf("DX%.3f%s", val, char(10)), "string");
    end
        end

        % Button pushed function: GnderButton_6
        function GnderButton_6Pushed(app, event)
  if ~isempty(app.com) && isvalid(app.com)
        val = app.Kp_yEditField.Value;
        flush(app.com, "output");
        write(app.com, sprintf("PY%.3f%s", val, char(10)), "string");
    end
        end

        % Button pushed function: GnderButton_7
        function GnderButton_7Pushed(app, event)
    if ~isempty(app.com) && isvalid(app.com)
        val = app.Ki_yEditField.Value;
        flush(app.com, "output");
        write(app.com, sprintf("IY%.3f%s", val, char(10)), "string");
    end
        end

        % Button pushed function: GnderButton_8
        function GnderButton_8Pushed(app, event)
 if ~isempty(app.com) && isvalid(app.com)
        val = app.Kd_yEditField.Value;
        flush(app.com, "output");
        write(app.com, sprintf("DY%.3f%s", val, char(10)), "string");
    end
        end

        % Value changed function: SetpointXSlider
        function SetpointXSliderValueChanged(app, event)
   value = app.SetpointXSlider.Value;
    app.SetpointXEditField.Value = value;
    if ~isempty(app.com) && isvalid(app.com)
        flush(app.com, "output");
        write(app.com, sprintf("X%.2f%s", value, char(10)), "string");
    end
        end

        % Value changed function: SetpointYSlider
        function SetpointYSliderValueChanged(app, event)
  value = app.SetpointYSlider.Value;
    app.SetpointYEditField.Value = value;
    if ~isempty(app.com) && isvalid(app.com)
        flush(app.com, "output");
        write(app.com, sprintf("Y%.2f%s", value, char(10)), "string");
    end
        end

        % Button pushed function: TemizleButton
        function TemizleButtonPushed(app, event)
            % X eksenindeki çizgileri temizle
    clearpoints(app.LineX);
    clearpoints(app.LineXSet);
    
    % X eksenini mevcut zamana göre hizalama
    t_now = seconds(datetime('now') - app.StartTime);
    app.UIAxes.XLim = [t_now, t_now + 100]; 
    drawnow;
        end

        % Button pushed function: TemizleButton_2
        function TemizleButton_2Pushed(app, event)
            % Y eksenindeki cizgileri temizleme
    clearpoints(app.LineY);
    clearpoints(app.LineYSet);
    
    % Y eksenini mevcut zamana göre hizalama
    t_now = seconds(datetime('now') - app.StartTime);
    app.UIAxes2.XLim = [t_now, t_now + 100];
    drawnow;
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 640 480];
            app.UIFigure.Name = 'MATLAB App';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Gerçek Zamanlı X Ekseni')
            xlabel(app.UIAxes, 'Zaman')
            ylabel(app.UIAxes, 'Açı')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Position = [12 263 300 185];

            % Create UIAxes2
            app.UIAxes2 = uiaxes(app.UIFigure);
            title(app.UIAxes2, 'Gerçek Zamanlı Y Ekseni')
            xlabel(app.UIAxes2, 'Zaman')
            ylabel(app.UIAxes2, 'Açı')
            zlabel(app.UIAxes2, 'Z')
            app.UIAxes2.Position = [326 263 300 185];

            % Create SetpointXEditFieldLabel
            app.SetpointXEditFieldLabel = uilabel(app.UIFigure);
            app.SetpointXEditFieldLabel.HorizontalAlignment = 'right';
            app.SetpointXEditFieldLabel.FontWeight = 'bold';
            app.SetpointXEditFieldLabel.Position = [15 218 64 22];
            app.SetpointXEditFieldLabel.Text = 'Setpoint X';

            % Create SetpointXEditField
            app.SetpointXEditField = uieditfield(app.UIFigure, 'numeric');
            app.SetpointXEditField.FontWeight = 'bold';
            app.SetpointXEditField.Position = [94 218 100 22];

            % Create SetpointYEditFieldLabel
            app.SetpointYEditFieldLabel = uilabel(app.UIFigure);
            app.SetpointYEditFieldLabel.HorizontalAlignment = 'right';
            app.SetpointYEditFieldLabel.FontWeight = 'bold';
            app.SetpointYEditFieldLabel.Position = [326 218 64 22];
            app.SetpointYEditFieldLabel.Text = 'Setpoint Y';

            % Create SetpointYEditField
            app.SetpointYEditField = uieditfield(app.UIFigure, 'numeric');
            app.SetpointYEditField.FontWeight = 'bold';
            app.SetpointYEditField.Position = [405 218 100 22];

            % Create GnderButton
            app.GnderButton = uibutton(app.UIFigure, 'push');
            app.GnderButton.ButtonPushedFcn = createCallbackFcn(app, @GnderButtonPushed, true);
            app.GnderButton.Position = [212 218 100 22];
            app.GnderButton.Text = 'Gönder';

            % Create GnderButton_2
            app.GnderButton_2 = uibutton(app.UIFigure, 'push');
            app.GnderButton_2.ButtonPushedFcn = createCallbackFcn(app, @GnderButton_2Pushed, true);
            app.GnderButton_2.Position = [519 218 100 22];
            app.GnderButton_2.Text = 'Gönder';

            % Create PORTDropDownLabel
            app.PORTDropDownLabel = uilabel(app.UIFigure);
            app.PORTDropDownLabel.HorizontalAlignment = 'right';
            app.PORTDropDownLabel.FontSize = 14;
            app.PORTDropDownLabel.Position = [12 16 44 22];
            app.PORTDropDownLabel.Text = 'PORT';

            % Create PORTDropDown
            app.PORTDropDown = uidropdown(app.UIFigure);
            app.PORTDropDown.Items = {'COM 1', 'COM 2', 'COM 3', 'COM 4', 'COM 5'};
            app.PORTDropDown.ValueChangedFcn = createCallbackFcn(app, @PORTDropDownValueChanged, true);
            app.PORTDropDown.FontSize = 14;
            app.PORTDropDown.Position = [71 16 100 22];
            app.PORTDropDown.Value = 'COM 1';

            % Create Kp_xEditFieldLabel
            app.Kp_xEditFieldLabel = uilabel(app.UIFigure);
            app.Kp_xEditFieldLabel.HorizontalAlignment = 'right';
            app.Kp_xEditFieldLabel.FontWeight = 'bold';
            app.Kp_xEditFieldLabel.Position = [44 183 35 22];
            app.Kp_xEditFieldLabel.Text = 'Kp_x';

            % Create Kp_xEditField
            app.Kp_xEditField = uieditfield(app.UIFigure, 'numeric');
            app.Kp_xEditField.FontWeight = 'bold';
            app.Kp_xEditField.Position = [94 183 100 22];

            % Create Ki_xEditFieldLabel
            app.Ki_xEditFieldLabel = uilabel(app.UIFigure);
            app.Ki_xEditFieldLabel.HorizontalAlignment = 'right';
            app.Ki_xEditFieldLabel.FontWeight = 'bold';
            app.Ki_xEditFieldLabel.Position = [48 150 31 22];
            app.Ki_xEditFieldLabel.Text = 'Ki_x';

            % Create Ki_xEditField
            app.Ki_xEditField = uieditfield(app.UIFigure, 'numeric');
            app.Ki_xEditField.FontWeight = 'bold';
            app.Ki_xEditField.Position = [94 150 100 22];

            % Create Kd_xEditFieldLabel
            app.Kd_xEditFieldLabel = uilabel(app.UIFigure);
            app.Kd_xEditFieldLabel.HorizontalAlignment = 'right';
            app.Kd_xEditFieldLabel.FontWeight = 'bold';
            app.Kd_xEditFieldLabel.Position = [44 114 35 22];
            app.Kd_xEditFieldLabel.Text = 'Kd_x';

            % Create Kd_xEditField
            app.Kd_xEditField = uieditfield(app.UIFigure, 'numeric');
            app.Kd_xEditField.FontWeight = 'bold';
            app.Kd_xEditField.Position = [94 114 100 22];

            % Create Kp_yEditFieldLabel
            app.Kp_yEditFieldLabel = uilabel(app.UIFigure);
            app.Kp_yEditFieldLabel.HorizontalAlignment = 'right';
            app.Kp_yEditFieldLabel.FontWeight = 'bold';
            app.Kp_yEditFieldLabel.Position = [355 183 35 22];
            app.Kp_yEditFieldLabel.Text = 'Kp_y';

            % Create Kp_yEditField
            app.Kp_yEditField = uieditfield(app.UIFigure, 'numeric');
            app.Kp_yEditField.FontWeight = 'bold';
            app.Kp_yEditField.Position = [405 183 100 22];

            % Create Ki_yEditFieldLabel
            app.Ki_yEditFieldLabel = uilabel(app.UIFigure);
            app.Ki_yEditFieldLabel.HorizontalAlignment = 'right';
            app.Ki_yEditFieldLabel.FontWeight = 'bold';
            app.Ki_yEditFieldLabel.Position = [359 150 31 22];
            app.Ki_yEditFieldLabel.Text = 'Ki_y';

            % Create Ki_yEditField
            app.Ki_yEditField = uieditfield(app.UIFigure, 'numeric');
            app.Ki_yEditField.FontWeight = 'bold';
            app.Ki_yEditField.Position = [405 150 100 22];

            % Create Kd_yEditFieldLabel
            app.Kd_yEditFieldLabel = uilabel(app.UIFigure);
            app.Kd_yEditFieldLabel.HorizontalAlignment = 'right';
            app.Kd_yEditFieldLabel.FontWeight = 'bold';
            app.Kd_yEditFieldLabel.Position = [355 114 35 22];
            app.Kd_yEditFieldLabel.Text = 'Kd_y';

            % Create Kd_yEditField
            app.Kd_yEditField = uieditfield(app.UIFigure, 'numeric');
            app.Kd_yEditField.FontWeight = 'bold';
            app.Kd_yEditField.Position = [405 114 100 22];

            % Create GnderButton_3
            app.GnderButton_3 = uibutton(app.UIFigure, 'push');
            app.GnderButton_3.ButtonPushedFcn = createCallbackFcn(app, @GnderButton_3Pushed, true);
            app.GnderButton_3.Position = [212 183 100 22];
            app.GnderButton_3.Text = 'Gönder';

            % Create GnderButton_4
            app.GnderButton_4 = uibutton(app.UIFigure, 'push');
            app.GnderButton_4.ButtonPushedFcn = createCallbackFcn(app, @GnderButton_4Pushed, true);
            app.GnderButton_4.Position = [212 150 100 22];
            app.GnderButton_4.Text = 'Gönder';

            % Create GnderButton_5
            app.GnderButton_5 = uibutton(app.UIFigure, 'push');
            app.GnderButton_5.ButtonPushedFcn = createCallbackFcn(app, @GnderButton_5Pushed, true);
            app.GnderButton_5.Position = [212 114 100 22];
            app.GnderButton_5.Text = 'Gönder';

            % Create GnderButton_6
            app.GnderButton_6 = uibutton(app.UIFigure, 'push');
            app.GnderButton_6.ButtonPushedFcn = createCallbackFcn(app, @GnderButton_6Pushed, true);
            app.GnderButton_6.Position = [519 183 100 22];
            app.GnderButton_6.Text = 'Gönder';

            % Create GnderButton_7
            app.GnderButton_7 = uibutton(app.UIFigure, 'push');
            app.GnderButton_7.ButtonPushedFcn = createCallbackFcn(app, @GnderButton_7Pushed, true);
            app.GnderButton_7.Position = [519 150 100 22];
            app.GnderButton_7.Text = 'Gönder';

            % Create GnderButton_8
            app.GnderButton_8 = uibutton(app.UIFigure, 'push');
            app.GnderButton_8.ButtonPushedFcn = createCallbackFcn(app, @GnderButton_8Pushed, true);
            app.GnderButton_8.Position = [519 114 100 22];
            app.GnderButton_8.Text = 'Gönder';

            % Create SetpointXSliderLabel
            app.SetpointXSliderLabel = uilabel(app.UIFigure);
            app.SetpointXSliderLabel.HorizontalAlignment = 'right';
            app.SetpointXSliderLabel.FontWeight = 'bold';
            app.SetpointXSliderLabel.Position = [15 76 64 22];
            app.SetpointXSliderLabel.Text = 'Setpoint X';

            % Create SetpointXSlider
            app.SetpointXSlider = uislider(app.UIFigure);
            app.SetpointXSlider.Limits = [0 180];
            app.SetpointXSlider.ValueChangedFcn = createCallbackFcn(app, @SetpointXSliderValueChanged, true);
            app.SetpointXSlider.FontWeight = 'bold';
            app.SetpointXSlider.Position = [101 85 150 3];

            % Create SetpointYSliderLabel
            app.SetpointYSliderLabel = uilabel(app.UIFigure);
            app.SetpointYSliderLabel.HorizontalAlignment = 'right';
            app.SetpointYSliderLabel.FontWeight = 'bold';
            app.SetpointYSliderLabel.Position = [326 76 64 22];
            app.SetpointYSliderLabel.Text = 'Setpoint Y';

            % Create SetpointYSlider
            app.SetpointYSlider = uislider(app.UIFigure);
            app.SetpointYSlider.Limits = [0 180];
            app.SetpointYSlider.ValueChangedFcn = createCallbackFcn(app, @SetpointYSliderValueChanged, true);
            app.SetpointYSlider.FontWeight = 'bold';
            app.SetpointYSlider.Position = [412 85 150 3];

            % Create TemizleButton
            app.TemizleButton = uibutton(app.UIFigure, 'push');
            app.TemizleButton.ButtonPushedFcn = createCallbackFcn(app, @TemizleButtonPushed, true);
            app.TemizleButton.Position = [12 250 100 22];
            app.TemizleButton.Text = 'Temizle';

            % Create TemizleButton_2
            app.TemizleButton_2 = uibutton(app.UIFigure, 'push');
            app.TemizleButton_2.ButtonPushedFcn = createCallbackFcn(app, @TemizleButton_2Pushed, true);
            app.TemizleButton_2.Position = [326 250 100 22];
            app.TemizleButton_2.Text = 'Temizle';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = ControlApp_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end