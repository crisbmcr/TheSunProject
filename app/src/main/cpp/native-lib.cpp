#include <jni.h>
#include <android/log.h>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/blenders.hpp>

#define LOG_TAG "PanoramaJNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static const char* stitchStatusToStr(cv::Stitcher::Status s) {
    switch (s) {
        case cv::Stitcher::OK: return "OK";
        case cv::Stitcher::ERR_NEED_MORE_IMGS: return "ERR_NEED_MORE_IMGS";
        case cv::Stitcher::ERR_HOMOGRAPHY_EST_FAIL: return "ERR_HOMOGRAPHY_EST_FAIL";
        case cv::Stitcher::ERR_CAMERA_PARAMS_ADJUST_FAIL: return "ERR_CAMERA_PARAMS_ADJUST_FAIL";
        default: return "UNKNOWN";
    }
}

static jbyteArray vecToJByteArray(JNIEnv* env, const std::vector<uchar>& v) {
    jbyteArray arr = env->NewByteArray((jsize)v.size());
    if (!arr) return nullptr;
    env->SetByteArrayRegion(arr, 0, (jsize)v.size(),
                            reinterpret_cast<const jbyte*>(v.data()));
    return arr;
}

// ---------------- EXIF ORIENTATION (simple, como venías usando) ----------------
static uint16_t rd16(const unsigned char* p, bool le) {
    return le ? (uint16_t)(p[0] | (p[1] << 8)) : (uint16_t)((p[0] << 8) | p[1]);
}
static uint32_t rd32(const unsigned char* p, bool le) {
    return le ? (uint32_t)(p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24))
              : (uint32_t)((p[0]<<24) | (p[1]<<16) | (p[2]<<8) | p[3]);
}

static int readExifOrientation(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return 1;

    std::vector<unsigned char> buf(64 * 1024);
    size_t n = fread(buf.data(), 1, buf.size(), f);
    fclose(f);
    if (n < 4) return 1;

    if (!(buf[0] == 0xFF && buf[1] == 0xD8)) return 1; // JPEG SOI

    size_t i = 2;
    while (i + 4 < n) {
        if (buf[i] != 0xFF) { i++; continue; }
        unsigned char marker = buf[i + 1];
        if (marker == 0xDA) break; // SOS

        uint16_t segLen = (uint16_t)((buf[i + 2] << 8) | buf[i + 3]);
        if (segLen < 2) break;

        size_t segStart = i + 4;
        size_t segEnd   = i + 2 + segLen;
        if (segEnd > n) break;

        if (marker == 0xE1 && (segStart + 6) < segEnd) {
            if (memcmp(buf.data() + segStart, "Exif\0\0", 6) == 0) {
                const unsigned char* tiff = buf.data() + segStart + 6;
                size_t tiffLen = segEnd - (segStart + 6);
                if (tiffLen < 8) return 1;

                bool le = (tiff[0] == 'I' && tiff[1] == 'I');
                bool be = (tiff[0] == 'M' && tiff[1] == 'M');
                if (!le && !be) return 1;

                uint16_t magic = rd16(tiff + 2, le);
                if (magic != 0x002A) return 1;

                uint32_t ifd0Off = rd32(tiff + 4, le);
                if (ifd0Off + 2 > tiffLen) return 1;

                const unsigned char* ifd0 = tiff + ifd0Off;
                uint16_t entries = rd16(ifd0, le);
                size_t need = 2 + (size_t)entries * 12;
                if (ifd0Off + need > tiffLen) return 1;

                for (uint16_t e = 0; e < entries; e++) {
                    const unsigned char* ent = ifd0 + 2 + (size_t)e * 12;
                    uint16_t tag = rd16(ent + 0, le);
                    if (tag == 0x0112) {
                        uint16_t type  = rd16(ent + 2, le);
                        uint32_t count = rd32(ent + 4, le);
                        if (type == 3 && count == 1) {
                            uint16_t val = rd16(ent + 8, le);
                            if (val >= 1 && val <= 8) return (int)val;
                        }
                        return 1;
                    }
                }
                return 1;
            }
        }

        i = segEnd;
    }
    return 1;
}

static void applyExifOrientation(cv::Mat& img, int ori) {
    if (img.empty()) return;
    switch (ori) {
        case 1: break;
        case 2: cv::flip(img, img, 1); break;
        case 3: cv::rotate(img, img, cv::ROTATE_180); break;
        case 4: cv::flip(img, img, 0); break;
        case 5: cv::transpose(img, img); cv::flip(img, img, 1); break;
        case 6: cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE); break;
        case 7: cv::transpose(img, img); cv::flip(img, img, 0); break;
        case 8: cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE); break;
        default: break;
    }
}

// ---------------- Helpers: mask, crop, rotate ----------------
static cv::UMat makeNeighborMaskUMat(int n, int radius, bool closeLoop) {
    cv::Mat m = cv::Mat::zeros(n, n, CV_8U);

    for (int i = 0; i < n; ++i) {
        int j0 = std::max(0, i - radius);
        int j1 = std::min(n - 1, i + radius);
        for (int j = j0; j <= j1; ++j) {
            if (i == j) continue;
            m.at<uchar>(i, j) = 1;
        }
    }
    if (closeLoop && n > 2) {
        m.at<uchar>(0, n - 1) = 1;
        m.at<uchar>(n - 1, 0) = 1;
    }

    cv::UMat u;
    m.copyTo(u);
    return u;
}

// recorte robusto: mayor componente conectado
static bool cropLargestComponentInPlace(cv::Mat& pano) {
    if (pano.empty()) return false;

    cv::Mat gray, bin;
    cv::cvtColor(pano, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, bin, 1, 255, cv::THRESH_BINARY);
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    int best = -1;
    double bestArea = 0.0;
    for (int i = 0; i < (int)contours.size(); i++) {
        double a = cv::contourArea(contours[i]);
        if (a > bestArea) { bestArea = a; best = i; }
    }
    if (best < 0) return false;

    cv::Rect bb = cv::boundingRect(contours[best]);
    if (bb.width < 128 || bb.height < 128) return false;

    pano = pano(bb).clone();
    return true;
}

static void rotatePanoIfNeeded(cv::Mat& pano) {
    if (pano.empty()) return;
    // Tu caso: sale mega-alto (h >> w). Lo rotamos.
    if (pano.rows > pano.cols * 2) {
        LOGW("rotatePanoIfNeeded: rotating pano 90 CCW (was %dx%d)", pano.cols, pano.rows);
        cv::rotate(pano, pano, cv::ROTATE_90_COUNTERCLOCKWISE);
        LOGW("rotatePanoIfNeeded: rotated pano now %dx%d", pano.cols, pano.rows);
    }
}

static double nonBlackFraction(const cv::Mat& pano) {
    if (pano.empty()) return 0.0;
    cv::Mat gray, mask;
    cv::cvtColor(pano, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);
    return (double)cv::countNonZero(mask) / (double)mask.total();
}

static bool acceptPano(const cv::Mat& pano, bool closeLoop) {
    if (pano.empty()) return false;
    if (pano.cols < 512 || pano.rows < 128) return false;

    const double fill = nonBlackFraction(pano);
    const double aspect = (double)pano.cols / (double)pano.rows;

    LOGI("acceptPano: fill=%.3f aspect=%.3f size=%dx%d closeLoop=%d",
         fill, aspect, pano.cols, pano.rows, (int)closeLoop);

    if (fill < 0.50) return false;

    // En panorama abierto esperamos imagen más ancha que alta
    if (!closeLoop && aspect < 1.15) return false;

    return true;
}

// ---------------- Bundle adjusters ----------------
static cv::Ptr<cv::detail::BundleAdjusterBase> makeReprojBA_refineFocalAndPP() {
    auto ba = cv::makePtr<cv::detail::BundleAdjusterReproj>();
    cv::Mat_<uchar> rm = cv::Mat::zeros(3, 3, CV_8U);
    rm(0,0) = 1; rm(1,1) = 1; // fx, fy
    rm(0,2) = 1; rm(1,2) = 1; // ppx, ppy
    ba->setRefinementMask(rm);
    ba->setConfThresh(0.8);
    return ba;
}

static cv::Ptr<cv::detail::BundleAdjusterBase> makeReprojBA_refineFocalOnly() {
    auto ba = cv::makePtr<cv::detail::BundleAdjusterReproj>();
    cv::Mat_<uchar> rm = cv::Mat::zeros(3, 3, CV_8U);
    rm(0,0) = 1; rm(1,1) = 1; // fx, fy
    ba->setRefinementMask(rm);
    ba->setConfThresh(0.5);
    return ba;
}

// “NoBA real”: no refinamos nada y confThresh 0 para que no “falle” por confianza
static cv::Ptr<cv::detail::BundleAdjusterBase> makeNoBa() {
    auto ba = cv::makePtr<cv::detail::BundleAdjusterReproj>();
    cv::Mat_<uchar> rm = cv::Mat::zeros(3, 3, CV_8U);
    ba->setRefinementMask(rm);
    ba->setConfThresh(0.0);
    return ba;
}

// ---------------- Stitcher config ----------------
static void configureStitcher(const cv::Ptr<cv::Stitcher>& stitcher, int nImages, bool closeLoop) {
    const int radius = 1;
    stitcher->setMatchingMask(makeNeighborMaskUMat(nImages, radius, closeLoop));

    stitcher->setRegistrationResol(cv::Stitcher::ORIG_RESOL);
    stitcher->setSeamEstimationResol(0.15);
    stitcher->setCompositingResol(cv::Stitcher::ORIG_RESOL);

    // Para un único anillo horizontal, el warper cilíndrico suele ser más estable
    stitcher->setWarper(cv::makePtr<cv::CylindricalWarper>());

    // Corrección de onda horizontal para enderezar el anillo
    stitcher->setWaveCorrection(true);
    stitcher->setWaveCorrectKind(cv::detail::WAVE_CORRECT_HORIZ);

    stitcher->setPanoConfidenceThresh(0.30);

    stitcher->setExposureCompensator(
            cv::detail::ExposureCompensator::createDefault(
                    cv::detail::ExposureCompensator::GAIN_BLOCKS
            )
    );

    stitcher->setSeamFinder(cv::makePtr<cv::detail::VoronoiSeamFinder>());

    auto fb = cv::makePtr<cv::detail::FeatherBlender>();
    fb->setSharpness(0.02f);
    stitcher->setBlender(fb);
}

static cv::Stitcher::Status runOnce(
        const std::vector<cv::Mat>& images,
        const char* label,
        const cv::Ptr<cv::detail::BundleAdjusterBase>& ba,
        bool closeLoop,
        bool cropBlack,
        cv::Mat& pano_out
) {
    cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);
    configureStitcher(stitcher, (int)images.size(), closeLoop);

    if (ba) stitcher->setBundleAdjuster(ba);

    cv::Stitcher::Status st = stitcher->estimateTransform(images);

    LOGI("stitchFromFiles: %s estimateTransform => status=%d (%s)",
         label, (int)st, stitchStatusToStr(st));

    if (st == cv::Stitcher::OK) {
        std::vector<int> comp = stitcher->component();
        LOGI("stitchFromFiles: %s component size=%zu", label, comp.size());
        for (size_t i = 0; i < comp.size(); ++i) {
            LOGI("stitchFromFiles: %s component[%zu]=%d", label, i, comp[i]);
        }
    }

    if (st != cv::Stitcher::OK) return st;

    cv::Mat pano;
    try {
        LOGI("stitchFromFiles: %s composePanorama START", label);
        int64 t0 = cv::getTickCount();
        st = stitcher->composePanorama(pano);
        double sec = (cv::getTickCount() - t0) / cv::getTickFrequency();
        LOGI("stitchFromFiles: %s composePanorama END st=%d (%s) time=%.2fs pano=%dx%d",
             label, (int)st, stitchStatusToStr(st), sec, pano.cols, pano.rows);
    } catch (const cv::Exception& e) {
        LOGE("stitchFromFiles: %s composePanorama threw: %s", label, e.what());
        return cv::Stitcher::ERR_HOMOGRAPHY_EST_FAIL;
    }

    if (st != cv::Stitcher::OK || pano.empty()) return st;

    // FIX clave: si sale mega-alto (tu caso), rotarlo
    rotatePanoIfNeeded(pano);

    if (cropBlack) {
        if (cropLargestComponentInPlace(pano)) {
            LOGI("stitchFromFiles: %s cropped pano=%dx%d", label, pano.cols, pano.rows);
        } else {
            LOGW("stitchFromFiles: %s crop failed (keeping original)", label);
        }
    }

    if (!acceptPano(pano, closeLoop)) {
        LOGW("stitchFromFiles: %s pano degenerado (%dx%d).", label, pano.cols, pano.rows);
        return cv::Stitcher::ERR_HOMOGRAPHY_EST_FAIL;
    }

    pano_out = pano;
    return cv::Stitcher::OK;
}

// ---------------- JNI ----------------
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_sunproject_NativeStitcher_checkStitcherAvailable(JNIEnv*, jobject) {
    try {
        cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);
        return stitcher ? JNI_TRUE : JNI_FALSE;
    } catch (...) {
        return JNI_FALSE;
    }
}

extern "C"
JNIEXPORT jbyteArray JNICALL
Java_com_example_sunproject_NativeStitcher_selfTestStitch(JNIEnv* env, jobject, jint width, jint height, jint jpegQuality) {
    try {
        int w = (width  > 0) ? width  : 1280;
        int h = (height > 0) ? height : 720;
        int q = (jpegQuality < 30) ? 30 : (jpegQuality > 100 ? 100 : jpegQuality);

        cv::Mat base(h, w * 2, CV_8UC3);
        cv::randu(base, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

        int overlap = w / 2;
        cv::Mat img1 = base(cv::Rect(0, 0, w, h)).clone();
        cv::Mat img2 = base(cv::Rect(w - overlap, 0, w, h)).clone();

        std::vector<cv::Mat> images = { img1, img2 };

        cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);
        cv::Mat pano;
        cv::Stitcher::Status st = stitcher->stitch(images, pano);

        LOGI("selfTestStitch: status=%d pano=%dx%d", (int)st, pano.cols, pano.rows);
        if (st != cv::Stitcher::OK || pano.empty()) return nullptr;

        std::vector<uchar> buf;
        std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, q };
        if (!cv::imencode(".jpg", pano, buf, params) || buf.empty()) return nullptr;

        return vecToJByteArray(env, buf);
    } catch (const cv::Exception& e) {
        LOGE("selfTestStitch cv::Exception: %s", e.what());
        return nullptr;
    } catch (...) {
        LOGE("selfTestStitch unknown exception");
        return nullptr;
    }
}

extern "C"
JNIEXPORT jbyteArray JNICALL
Java_com_example_sunproject_NativeStitcher_stitchFromFiles(
        JNIEnv* env, jobject,
        jobjectArray jpaths, jint jpegQuality, jint maxDim,
        jboolean jCloseLoop
) {
    bool closeLoop = (jCloseLoop == JNI_TRUE);

    try {
        if (!jpaths) return nullptr;

        jsize n = env->GetArrayLength(jpaths);
        if (n < 2) {
            LOGE("stitchFromFiles: need at least 2 images, got %d", (int)n);
            return nullptr;
        }

        int q  = (jpegQuality < 30) ? 30 : (jpegQuality > 100 ? 100 : jpegQuality);
        int md = (maxDim <= 0) ? 0 : (int)maxDim;

        std::vector<cv::Mat> regImages;
        regImages.reserve((size_t)n);

        for (jsize i = 0; i < n; i++) {
            jstring jpath = (jstring)env->GetObjectArrayElement(jpaths, i);
            if (!jpath) continue;

            const char* cpath = env->GetStringUTFChars(jpath, nullptr);
            std::string path = cpath ? cpath : "";
            env->ReleaseStringUTFChars(jpath, cpath);
            env->DeleteLocalRef(jpath);

            if (path.empty()) continue;

            // ORIGINAL: se carga completa
            cv::Mat imgOriginal = cv::imread(path, cv::IMREAD_COLOR);
            if (imgOriginal.empty()) {
                LOGE("stitchFromFiles: imread FAILED for %s", path.c_str());
                return nullptr;
            }

            int ori = readExifOrientation(path);
            applyExifOrientation(imgOriginal, ori);

            LOGI("stitchFromFiles: loaded ORIGINAL ori=%d w=%d h=%d",
                 ori, imgOriginal.cols, imgOriginal.rows);

            // WORK: copia para el registro/cosido del anillo horizontal
            cv::Mat imgWork = imgOriginal.clone();

            if (md > 0) {
                int maxSide = std::max(imgWork.cols, imgWork.rows);
                if (maxSide > md) {
                    double s = (double)md / (double)maxSide;
                    cv::resize(imgWork, imgWork, cv::Size(), s, s, cv::INTER_AREA);
                    LOGI("stitchFromFiles: WORK resized to w=%d h=%d", imgWork.cols, imgWork.rows);
                }
            }

            // Recorte solo sobre la imagen de trabajo.
            // La original guardada en disco NO se modifica.
            if (imgWork.rows >= 400) {
                const int cropTopPct = 10;
                const int cropBottomPct = 30;

                int y0 = imgWork.rows * cropTopPct / 100;
                int keepH = imgWork.rows * (100 - cropTopPct - cropBottomPct) / 100;

                imgWork = imgWork(cv::Rect(0, y0, imgWork.cols, keepH)).clone();
                LOGI("stitchFromFiles: WORK crop top=%d%% bottom=%d%% -> w=%d h=%d",
                     cropTopPct, cropBottomPct, imgWork.cols, imgWork.rows);
            }

            regImages.push_back(imgWork);
        }

        if ((int)regImages.size() < 2) return nullptr;

        cv::Mat pano;
        cv::Stitcher::Status st = cv::Stitcher::ERR_HOMOGRAPHY_EST_FAIL;

        // Loop: primero NoBA (en móviles suele ser más estable para 360)
        if (closeLoop) {
            st = runOnce(regImages, "PANORAMA[Loop-NoBA]", makeNoBa(), true, true, pano);
            if (st != cv::Stitcher::OK) {
                st = runOnce(regImages, "PANORAMA[Loop-BA-Focal]", makeReprojBA_refineFocalOnly(), true, true, pano);
            }
        } else {
            st = runOnce(regImages, "PANORAMA[NoBA]", makeNoBa(), false, true, pano);
            if (st != cv::Stitcher::OK) {
                st = runOnce(regImages, "PANORAMA[BA-Focal]", makeReprojBA_refineFocalOnly(), false, true, pano);
            }
        }

        if (st != cv::Stitcher::OK) {
            LOGE("stitchFromFiles: FINAL FAIL status=%d (%s)", (int)st, stitchStatusToStr(st));
            return nullptr;
        }

        std::vector<uchar> buf;
        std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, q };
        if (!cv::imencode(".jpg", pano, buf, params) || buf.empty()) {
            LOGE("stitchFromFiles: imencode failed");
            return nullptr;
        }

        return vecToJByteArray(env, buf);

    } catch (const cv::Exception& e) {
        LOGE("stitchFromFiles cv::Exception: %s", e.what());
        return nullptr;
    } catch (...) {
        LOGE("stitchFromFiles unknown exception");
        return nullptr;
    }
}