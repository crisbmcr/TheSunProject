package com.example.sunproject

import android.net.Uri
import android.os.Bundle
import android.widget.ImageView
import androidx.appcompat.app.AppCompatActivity

class AnalysisActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_analysis)

        val panoramaImageView: ImageView = findViewById(R.id.panoramaImageView)
        val imagePath = intent.getStringExtra("panorama_path")

        if (imagePath != null) {
            panoramaImageView.setImageURI(Uri.parse(imagePath))
        }
    }
}
