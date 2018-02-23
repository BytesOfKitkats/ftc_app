package org.firstinspires.ftc.robotcontroller.internal;

import android.os.Bundle;
import android.support.annotation.Nullable;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.ThemedActivity;

import java.io.File;

/**
 * Created by shiv on 2/23/2018.
 */

public class FtcRedSettingsActivity extends ThemedActivity {
    @Override public String getTag() { return this.getClass().getSimpleName(); }

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.red_settings);
        final Button btnSave = (Button)findViewById(R.id.redSave);
        final CheckBox chkRight = (CheckBox)findViewById(R.id.cbRedRight);
        final EditText angRight = (EditText)findViewById(R.id.textRedRight);
        final CheckBox chkCenter = (CheckBox)findViewById(R.id.cbRedCenter);
        final EditText angCenter = (EditText)findViewById(R.id.textRedCenter);
        final CheckBox chkLeft = (CheckBox)findViewById(R.id.cbRedLeft);
        final EditText angLeft = (EditText)findViewById(R.id.textRedLeft);

        File file = AppUtil.getInstance().getSettingsFile("BoKRedAngles.txt");
        String value = ReadWriteFile.readFile(file);
        if (value.isEmpty()) {

        }
        else {
            //Log.v("BOK", "Read: " + value);
            String[] tokens = value.split(",");
            chkRight.setChecked(Boolean.parseBoolean(tokens[0]));
            angRight.setText(tokens[1]);
            chkCenter.setChecked(Boolean.parseBoolean(tokens[2]));
            angCenter.setText(tokens[3]);
            chkLeft.setChecked(Boolean.parseBoolean(tokens[4]));
            angLeft.setText(tokens[5]);
        }

        btnSave.setOnClickListener(new View.OnClickListener() {
           public void onClick(View v) {
               File file = AppUtil.getInstance().getSettingsFile("BoKRedAngles.txt");

               String fileText = Boolean.toString(chkRight.isChecked());
               fileText += "," + angRight.getText();
               fileText += "," + Boolean.toString(chkCenter.isChecked());
               fileText += "," + angCenter.getText();
               fileText += "," + Boolean.toString(chkLeft.isChecked());
               fileText += "," + angLeft.getText();
               ReadWriteFile.writeFile(file,fileText);
           }
        });
    }
}
