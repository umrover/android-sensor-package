package org.mrover.sensorpackage;

import android.Manifest;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
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
import android.widget.TextView;

import java.io.IOException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class MainActivity extends Activity implements SensorEventListener, LocationListener {
    private static final String TAG = "MainActivity";

    private SensorManager sensor_service_;
    private LocationManager gps_service_;

    private UsbManager usb_service_;

    private AccessoryProtocol protocol_;
    private Thread tx_thread_;

    private TextView connection_status_;
    private TextView lat_;
    private TextView lon_;
    private TextView azimuth_;

    private int lat_deg_;
    private float lat_min_;
    private int lon_deg_;
    private float lon_min_;
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

    @Override
    public synchronized void onSensorChanged(SensorEvent evt) {
        if (evt.sensor.getType() == Sensor.TYPE_ORIENTATION) {
            // TODO check this math
            float azimuth = evt.values[0];
            if (azimuth < 0.0f) {
                azimuth += 360.0f;
            }
            Log.d(TAG, "new azimuthal angle");
            this.mutex_.lock();
            try {
                this.bearing_deg_ = azimuth;
                this.bearing_valid_ = true;
            } finally {
                this.mutex_.unlock();
            }
            this.azimuth_.setText(String.format("%.2f deg", azimuth));
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
}