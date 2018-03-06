package org.firstinspires.ftc.robotcontroller.internal;

import android.os.Bundle;
import android.support.annotation.Nullable;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.SeekBar;
import android.widget.TextView;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.ThemedActivity;

import java.io.File;

/**
 * Created by shiv on 2/23/2018.
 */

public class FtcGlyph2SettingsActivity extends ThemedActivity {
    @Override public String getTag() { return this.getClass().getSimpleName(); }

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.glyph2_settings);
        final Button btnSave = (Button)findViewById(R.id.glyphSave);
        final CheckBox chkRight = (CheckBox)findViewById(R.id.cbRight);
        final TextView angRightVal = (TextView)findViewById(R.id.textViewRight);
        final SeekBar angRight = (SeekBar)findViewById(R.id.seekBarRight);
        final TextView angCenterVal = (TextView)findViewById(R.id.textViewCenter);
        final SeekBar angCenter = (SeekBar)findViewById(R.id.seekBarCenter);
        final CheckBox chkCenter = (CheckBox)findViewById(R.id.cbCenter);
        final TextView angLeftVal = (TextView)findViewById(R.id.textViewLeft);
        final SeekBar angLeft = (SeekBar)findViewById(R.id.seekBarLeft);
        final CheckBox chkLeft = (CheckBox)findViewById(R.id.cbLeft);

        angRight.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            int progressChangedValue = 0;

            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChangedValue = (progress-20);
                angRightVal.setText(Integer.toString(progressChangedValue));
            }

            public void onStartTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub
            }

            public void onStopTrackingTouch(SeekBar seekBar) {
                angRightVal.setText(Integer.toString(progressChangedValue));
            }
        });
        angCenter.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            int progressChangedValue = 0;

            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChangedValue = (progress-20);
                angCenterVal.setText(Integer.toString(progressChangedValue));
            }

            public void onStartTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub
            }

            public void onStopTrackingTouch(SeekBar seekBar) {
                angCenterVal.setText(Integer.toString(progressChangedValue));
            }
        });
        angLeft.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            int progressChangedValue = 0;

            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChangedValue = (progress-20);
                angLeftVal.setText(Integer.toString(progressChangedValue));
            }

            public void onStartTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub
            }

            public void onStopTrackingTouch(SeekBar seekBar) {
                angLeftVal.setText(Integer.toString(progressChangedValue));
            }
        });

        File file = AppUtil.getInstance().getSettingsFile("BoKAngles.txt");
        String value = ReadWriteFile.readFile(file);
        if (value.isEmpty()) {

        }
        else {
            //Log.v("BOK", "Read: " + value);
            String[] tokens = value.split(",");
            chkRight.setChecked(Boolean.parseBoolean(tokens[0]));
            angRight.setProgress(Integer.parseInt(tokens[1])+20);
            angRightVal.setText(tokens[1]);
            chkCenter.setChecked(Boolean.parseBoolean(tokens[2]));
            angCenter.setProgress(Integer.parseInt(tokens[3])+20);
            angCenterVal.setText(tokens[3]);
            chkLeft.setChecked(Boolean.parseBoolean(tokens[4]));
            angLeft.setProgress(Integer.parseInt(tokens[5])+20);
            angLeftVal.setText(tokens[5]);
        }

        btnSave.setOnClickListener(new View.OnClickListener() {
           public void onClick(View v) {
               File file = AppUtil.getInstance().getSettingsFile("BoKAngles.txt");

               String fileText = Boolean.toString(chkRight.isChecked());
               fileText += "," + angRightVal.getText();
               fileText += "," + Boolean.toString(chkCenter.isChecked());
               fileText += "," + angCenterVal.getText();
               fileText += "," + Boolean.toString(chkLeft.isChecked());
               fileText += "," + angLeftVal.getText();
               ReadWriteFile.writeFile(file,fileText);
           }
        });
    }
}
