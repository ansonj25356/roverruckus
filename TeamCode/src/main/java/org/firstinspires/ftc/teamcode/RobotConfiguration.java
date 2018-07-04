package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

/**
 * Created by Shishir on 9/21/2017.
 */

public class RobotConfiguration {

    private Gamepad _gamepad;
    private Telemetry _telemetry;
    private AllianceColor _allianceColor;


    public enum AllianceColor
    {
        Blue,
        Red
    }

    public RobotConfiguration(Gamepad gamepad, Telemetry telemetry)
    {
        _gamepad = gamepad;
        _telemetry = telemetry;
        _allianceColor = AllianceColor.Blue;
    }

    public AllianceColor getAllianceColor() {
        return _allianceColor;
    }

    public void ShowMenu() {
        do {
            if (_gamepad.x) {
                _allianceColor = AllianceColor.Blue;
            }

            if (_gamepad.b) {
                _allianceColor = AllianceColor.Red;
            }


            _telemetry.addData("Menu", "x = Blue, b = Red,");
            _telemetry.addData("Alliance color is", "%s", _allianceColor);
            _telemetry.update();
        } while (true);






    }



}
