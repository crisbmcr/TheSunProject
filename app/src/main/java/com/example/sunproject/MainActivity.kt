package com.example.sunproject

import android.content.Intent
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.example.sunproject.databinding.ActivityMainBinding
import com.google.android.material.floatingactionbutton.FloatingActionButton
import android.util.Log
import com.example.sunproject.NativeProbe

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        val isAvailable = NativeStitcher.checkStitcherAvailable()
        Log.i("STITCHER_TEST", "Stitcher disponible: $isAvailable")


        // La única lógica necesaria aquí es configurar el botón.
        // Asegúrate de que tu layout 'activity_main.xml' tiene un botón con el id 'startCaptureButton'.
        val startCaptureButton: FloatingActionButton = findViewById(R.id.startCaptureButton)
        startCaptureButton.setOnClickListener {
            // Crea un Intent para abrir la CaptureActivity
            val intent = Intent(this, CaptureActivity::class.java)
            startActivity(intent)
        }
    }
}
