package org.firstinspires.ftc.robotcontroller.internal;

import android.os.Bundle;
import android.support.annotation.Nullable;
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

public class FtcBlueSettingsActivity extends ThemedActivity {
    @Override public String getTag() { return this.getClass().getSimpleName(); }

    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.blue_settings);
        final Button btnSave = (Button)findViewById(R.id.blueSave);
        final CheckBox chkRight = (CheckBox)findViewById(R.id.cbBlueRight);
        final EditText angRight = (EditText)findViewById(R.id.textBlueRight);
        final CheckBox chkCenter = (CheckBox)findViewById(R.id.cbBlueCenter);
        final EditText angCenter = (EditText)findViewById(R.id.textBlueCenter);
        final CheckBox chkLeft = (CheckBox)findViewById(R.id.cbBlueLeft);
        final EditText angLeft = (EditText)findViewById(R.id.textBlueLeft);

        File file = AppUtil.getInstance().getSettingsFile("BoKBlueAngles.txt");
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
                File file = AppUtil.getInstance().getSettingsFile("BoKBlueAngles.txt");

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
