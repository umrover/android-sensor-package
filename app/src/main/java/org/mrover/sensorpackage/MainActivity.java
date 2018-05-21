package org.mrover.sensorpackage;

import android.Manifest;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.hardware.GeomagneticField;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.location.LocationProvider;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.WindowManager;
import android.view.Surface;
import android.widget.TextView;

import java.io.IOException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class MainActivity extends Activity implements SensorEventListener, LocationListener {
    private static final String TAG = "MainActivity";

    private final float[] last_accel_ = new float[3];
    private final float[] last_mag_ = new float[3];

    private final float[] rot_matrix_a_ = new float[9];
    private final float[] rot_matrix_b_ = new float[9];
    private final float[] orientation_ = new float[3];

    private boolean last_accel_set_ = false;
    private boolean last_mag_set_ = false;

    private SensorManager sensor_service_;
    private LocationManager gps_service_;

    private UsbManager usb_service_;

    private AccessoryProtocol protocol_;
    private Thread tx_thread_;

    private TextView connection_status_;
    private TextView lat_;
    private TextView lon_;
    private TextView azimuth_;

    private GeomagneticField field_;

    private int lat_deg_;
    private float lat_min_;
    private int lon_deg_;
    private float lon_min_;
    //private ExponentialMovingAverage bearing_deg_ = new ExponentialMovingAverage(0.2f);
    private float bearing_deg_;
    private int num_sats_;
    private boolean gps_valid_ = false;
    private boolean bearing_valid_ = false;
    private Lock mutex_ = new ReentrantLock();

    private Runnable transmitter_ = new Runnable() {
        @Override
        public void run() {
            Log.d(TAG, "tx starting");
            while (true) {
                mutex_.lock();
                try {
                    if (protocol_.connected()) {
                        boolean valid = gps_valid_ && bearing_valid_;
                        Log.d(TAG, String.format("tx: %d deg %.3f min by %d deg %.3f min with bearing %.2f and %d sats (%d)",
                                lat_deg_, lat_min_, lon_deg_, lon_min_, bearing_deg_, num_sats_,
                                valid ? 1 : 0));
                        protocol_.transmit(lat_deg_, lat_min_,
                                lon_deg_, lon_min_, bearing_deg_,
                                num_sats_, valid);
                    }
                } catch (IOException e) {
                    Log.d(TAG, "IOException", e);
                } finally {
                    mutex_.unlock();
                }
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.setContentView(R.layout.activity_main);
        this.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        this.connection_status_ = (TextView) this.findViewById(R.id.connectedStatus);
        this.lat_ = (TextView) this.findViewById(R.id.gpsLatitude);
        this.lon_ = (TextView) this.findViewById(R.id.gpsLongitude);
        this.azimuth_ = (TextView) this.findViewById(R.id.azimuthalAngle);

        this.gps_service_ = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);
        this.sensor_service_ = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        this.usb_service_ = (UsbManager) this.getSystemService(Context.USB_SERVICE);

        this.protocol_ = new AccessoryProtocol(this.usb_service_);

        Intent intent = this.getIntent();
        if (intent.getAction().equals(UsbManager.ACTION_USB_ACCESSORY_ATTACHED)) {
            // We are connected
            UsbAccessory a = (UsbAccessory) intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
            if (this.protocol_.connect(a)) {
                this.connection_status_.setText("Connected");

                // Start transmit timer
                this.tx_thread_ = new Thread(this.transmitter_);
                this.tx_thread_.start();
            } else {
                this.connection_status_.setText("Failed");
            }
            // Prepare for possibility of detaching
            BroadcastReceiver closer = new BroadcastReceiver() {
                @Override
                public void onReceive(Context context, Intent intent) {
                    if (intent.getAction().equals(UsbManager.ACTION_USB_ACCESSORY_DETACHED)) {
                        MainActivity.this.protocol_.disconnect();
                        MainActivity.this.connection_status_.setText("Disconnected");
                        MainActivity.this.unregisterReceiver(this);
                        MainActivity.this.tx_thread_.interrupt();
                        // Terminate app
                        MainActivity.this.finish();
                    }
                }
            };
            IntentFilter filter = new IntentFilter();
            filter.addAction(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
            this.registerReceiver(closer, filter);
        }
        // If we aren't connected on start, just ignore, a new activity instance will be
        // started where we are connected.
        // TODO maybe a better way to do this?
    }

    @Override
    protected void onResume() {
        super.onResume();

        this.sensor_service_.registerListener(this,
                this.sensor_service_.getDefaultSensor(Sensor.TYPE_ORIENTATION),
                SensorManager.SENSOR_DELAY_NORMAL);
        /*this.sensor_service_.registerListener(this,
                this.sensor_service_.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_GAME);
        this.sensor_service_.registerListener(this,
                this.sensor_service_.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_GAME);*/
        /*this.sensor_service_.registerListener(this,
                this.sensor_service_.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR),
                SensorManager.SENSOR_DELAY_NORMAL);*/

        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    Activity#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for Activity#requestPermissions for more details.
            return;
        }
        this.gps_service_.requestLocationUpdates(LocationManager.GPS_PROVIDER,
                100, 0.5f, this);
    }

    @Override
    protected void onPause() {
        super.onPause();

        this.sensor_service_.unregisterListener(this);
    }

    private float mod(float a, float b) {
        return (a % b + b) % b;
    }

    private void set_azimuth(float azimuth) {
        azimuth = (float) Math.toDegrees(azimuth);
        if (this.field_ != null) {
            azimuth += this.field_.getDeclination();
        }
        azimuth = mod(azimuth, 360.0f);
        this.mutex_.lock();
        try {
            //azimuth = this.bearing_deg_.feed(azimuth);
            this.bearing_deg_ = azimuth;
            this.bearing_valid_ = true;
        } finally {
            this.mutex_.unlock();
        }
        this.azimuth_.setText(String.format("%.2f deg", azimuth));
    }

    @Override
    public void onSensorChanged(SensorEvent evt) {
        /*if (evt.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) {
            Log.d(TAG, "sensor is too unreliable to use");
            return;
        }*/
        if (evt.sensor.getType() == Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR) {
            SensorManager.getRotationMatrixFromVector(this.rot_matrix_a_, evt.values);
            int rot = this.getWindowManager().getDefaultDisplay().getRotation();
            switch (rot) {
                case Surface.ROTATION_0:
                    SensorManager.getOrientation(this.rot_matrix_a_, this.orientation_);
                    break;
                case Surface.ROTATION_90:
                    SensorManager.remapCoordinateSystem(this.rot_matrix_a_,
                            SensorManager.AXIS_Y, SensorManager.AXIS_MINUS_X,
                            this.rot_matrix_b_);
                    SensorManager.getOrientation(this.rot_matrix_b_, this.orientation_);
                    break;
                case Surface.ROTATION_180:
                    SensorManager.remapCoordinateSystem(this.rot_matrix_a_,
                            SensorManager.AXIS_MINUS_X, SensorManager.AXIS_MINUS_Y,
                            this.rot_matrix_b_);
                    SensorManager.getOrientation(this.rot_matrix_b_, this.orientation_);
                    break;
                case Surface.ROTATION_270:
                    SensorManager.remapCoordinateSystem(this.rot_matrix_a_,
                            SensorManager.AXIS_MINUS_Y, SensorManager.AXIS_X,
                            this.rot_matrix_b_);
                    SensorManager.getOrientation(this.rot_matrix_b_, this.orientation_);
                    break;
                default:
                    SensorManager.getOrientation(this.rot_matrix_a_, this.orientation_);
                    break;
            }
            this.set_azimuth(this.orientation_[0]);
        } else if (evt.sensor.getType() == Sensor.TYPE_ORIENTATION) {
            this.set_azimuth((float) Math.toRadians(evt.values[0]));
        } else if (evt.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            if (evt.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) {
                Log.d(TAG, "accelerometer unreliable");
            }
            System.arraycopy(evt.values, 0, this.last_accel_, 0, evt.values.length);
            this.last_accel_set_ = true;
        } else if (evt.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            if (evt.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) {
                Log.d(TAG, "magnetometer unreliable");
            }
            System.arraycopy(evt.values, 0, this.last_mag_, 0, evt.values.length);
            this.last_mag_set_ = true;
        }

        if (this.last_accel_set_ && this.last_mag_set_) {
            SensorManager.getRotationMatrix(this.rot_matrix_a_, null, this.last_accel_, this.last_mag_);
            /*SensorManager.remapCoordinateSystem(this.rot_matrix_a_,
                    SensorManager.AXIS_X, SensorManager.AXIS_Z,
                    this.rot_matrix_b_);*/
            SensorManager.getOrientation(this.rot_matrix_a_, this.orientation_);
            this.set_azimuth(this.orientation_[0]);
            this.last_accel_set_ = false;
            this.last_mag_set_ = false;
        }
    }

    // TODO no idea
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    private void gps_not_available() {
        this.lat_.setText("XX deg XX.XXX min");
        this.lon_.setText("XX deg XX.XXX min");

        Log.d(TAG, "GPS no longer valid");
        this.mutex_.lock();
        try {
            this.gps_valid_ = false;
        } finally {
            this.mutex_.unlock();
        }
    }

    @Override
    public void onLocationChanged(Location location) {
        double lat = location.getLatitude();
        double lon = location.getLongitude();
        int lat_deg = (int)Math.floor(lat);
        if (lat < 0) {
            lat_deg = (int)Math.ceil(lat);
        }
        int lon_deg = (int)Math.floor(lon);
        if (lon < 0) {
            lon_deg = (int)Math.ceil(lon);
        }

        double lat_min = (lat - lat_deg)*60.0;
        double lon_min = (lon - lon_deg)*60.0;

        this.lat_.setText(String.format(
                "%3d deg %2.3f min",
                lat_deg, lat_min
        ));
        this.lon_.setText(String.format(
            "%3d deg %2.3f min",
            lon_deg, lon_min
        ));

        this.field_ = new GeomagneticField(
                (float)lat, (float) lon,
                (float) location.getAltitude(),
                location.getTime());

        Log.d(TAG, "new GPS coordinates");
        this.mutex_.lock();
        try {
            this.gps_valid_ = true;
            this.lat_deg_ = lat_deg;
            this.lat_min_ = (float) lat_min;
            this.lon_deg_ = lon_deg;
            this.lon_min_ = (float) lon_min;
        } finally {
            this.mutex_.unlock();
        }
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
        if (status != LocationProvider.AVAILABLE) {
            this.gps_not_available();
        } else {
            this.mutex_.lock();
            Log.d(TAG, "GPS is now valid");
            try {
                this.gps_valid_ = true;
                if (extras.containsKey("satellites")) {
                    this.num_sats_ = extras.getInt("satellites");
                }
            } finally {
                this.mutex_.unlock();
            }
        }
    }

    @Override
    public void onProviderEnabled(String provider) {}

    @Override
    public void onProviderDisabled(String provider) {
        if (provider.equals(LocationManager.GPS_PROVIDER)) {
            this.gps_not_available();
        }
    }

    private static class ExponentialMovingAverage {
        private float alpha_;
        private float old_value_;
        private boolean is_valid_ = false;

        public ExponentialMovingAverage(float alpha) {
            this.alpha_ = alpha;
        }

        public float feed(float val) {
            if (!this.is_valid_) {
                this.old_value_ = val;
                this.is_valid_ = true;
                return val;
            }
            float new_val = this.old_value_ + this.alpha_*(val - this.old_value_);
            this.old_value_ = new_val;
            return new_val;
        }

        public float get() {
            return this.old_value_;
        }
    }
}
