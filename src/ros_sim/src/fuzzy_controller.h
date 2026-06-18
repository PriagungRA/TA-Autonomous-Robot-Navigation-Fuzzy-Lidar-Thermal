// ============================================================================
//  fuzzy_controller.h  --  Kontroler Fuzzy Mamdani (mandiri, header-only)
// ----------------------------------------------------------------------------
//  Port C++ dari desain scikit-fuzzy (BAB 3 skripsi). Meniru PERSIS perilaku
//  scikit-fuzzy ControlSystem:
//    - AND antaranteseden      = min
//    - implikasi Mamdani        = min (clip MF konsekuen pada kekuatan aturan)
//    - agregasi antar aturan    = max
//    - defuzzifikasi            = centroid pada universe ter-UPSAMPLE
//      (universe dasar step 0.01 + titik-potong tepat tempat MF memotong level
//       aktivasi, persis _interp_universe_fast milik skfuzzy), lalu integrasi
//       area piecewise-linear EKSAK (segitiga/persegi/trapesium) -- algoritma
//       skfuzzy.defuzzify.centroid yang sebenarnya, BUKAN sum(x*mf)/sum(mf).
//
//  Universe dasar (sama dgn scikit-fuzzy default np.arange step 0.01):
//    jarak     : 0.00 .. 2.00   (201 titik)
//    trav      : 0.00 .. 1.00   (101 titik)
//    kecepatan : 0.00 .. 1.00   (101 titik)
//    gain      : 0.50 .. 2.50   (201 titik)
//
//  Terverifikasi IDENTIK dengan scikit-fuzzy 0.5.0 pada grid padat 588 titik
//  (max|Delta kecepatan| = 5e-6, max|Delta gain| = 5e-6). Universe gain meniru
//  float-drift numpy.arange (titik akhir 2.5000000000000018) sehingga centroid
//  region 'kuat' sama persis. 4 titik uji skripsi (presisi penuh):
//    (jarak 0.50, trav 0.25) -> kecepatan 0.159, gain 2.131
//    (jarak 1.60, trav 0.90) -> kecepatan 0.844, gain 0.811
//    (jarak 0.80, trav 0.55) -> kecepatan 0.500, gain 1.500
//    (jarak 0.40, trav 0.85) -> kecepatan 0.500, gain 1.500
//
//  Pemakaian:
//      #include "fuzzy_controller.h"
//      double kec_scale, gain_belok;
//      fuzzy::fuzzyCompute(jarak, trav, kec_scale, gain_belok);
//      // kec_scale  in [0..1]      -> kalikan ke max_vel
//      // gain_belok in [0.5..2.5]  -> pengganti rep_gain crisp
// ============================================================================
#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

#include <algorithm>
#include <cmath>
#include <vector>

namespace fuzzy {

// ----------------------------------------------------------------------------
//  Fungsi keanggotaan dasar (sama dgn skfuzzy.trimf / trapmf, termasuk
//  penanganan tepi vertikal a==b atau c==d).
// ----------------------------------------------------------------------------
inline double trimf(double x, double a, double b, double c) {
    double y = 0.0;
    if (a != b && x > a && x < b) y = (x - a) / (b - a);
    else if (b != c && x > b && x < c) y = (c - x) / (c - b);
    if (x == b) y = 1.0;
    return y;
}

inline double trapmf(double x, double a, double b, double c, double d) {
    double y = 0.0;
    if (x >= b && x <= c) y = 1.0;                       // datar di puncak
    else if (a != b && x > a && x < b) y = (x - a) / (b - a); // naik
    else if (c != d && x > c && x < d) y = (d - x) / (d - c); // turun
    return y;
}

// ----------------------------------------------------------------------------
//  Keanggotaan tiap label (parameter PERSIS desain BAB 3)
// ----------------------------------------------------------------------------
// INPUT jarak (m)
inline double j_dekat (double x){ return trapmf(x, 0.0, 0.0, 0.30, 0.60); }
inline double j_sedang(double x){ return trimf (x, 0.40, 0.80, 1.20);     }
inline double j_jauh  (double x){ return trapmf(x, 1.00, 1.40, 2.0, 2.0); }
// INPUT trav (0..1)
inline double t_rendah(double x){ return trapmf(x, 0.0, 0.0, 0.25, 0.45); }
inline double t_sedang(double x){ return trimf (x, 0.35, 0.55, 0.75);     }
inline double t_tinggi(double x){ return trapmf(x, 0.60, 0.80, 1.0, 1.0); }
// OUTPUT kecepatan (0..1)
inline double k_lambat(double x){ return trapmf(x, 0.0, 0.0, 0.15, 0.35); }
inline double k_sedang(double x){ return trimf (x, 0.30, 0.50, 0.70);     }
inline double k_cepat (double x){ return trapmf(x, 0.60, 0.80, 1.0, 1.0); }
// OUTPUT gain (0.5..2.5)
inline double g_lemah (double x){ return trapmf(x, 0.5, 0.5, 0.9, 1.3);   }
inline double g_sedang(double x){ return trimf (x, 1.1, 1.5, 1.9);        }
inline double g_kuat  (double x){ return trapmf(x, 1.7, 2.1, 2.5, 2.5);   }

typedef double (*MF)(double);

// ----------------------------------------------------------------------------
//  Interpolasi membership linear pada grid dasar (spt skfuzzy interp_membership)
// ----------------------------------------------------------------------------
inline double interpMF(const std::vector<double>& gx,
                       const std::vector<double>& gmf, double x) {
    int n = (int)gx.size();
    if (x <= gx[0])   return gmf[0];
    if (x >= gx[n-1]) return gmf[n-1];
    // grid seragam -> indeks langsung
    double step = gx[1] - gx[0];
    int i = (int)std::floor((x - gx[0]) / step);
    if (i < 0) i = 0; if (i > n-2) i = n-2;
    double t = (x - gx[i]) / (gx[i+1] - gx[i]);
    return gmf[i] + t * (gmf[i+1] - gmf[i]);
}

// ----------------------------------------------------------------------------
//  Titik-potong universe pada level cut (replika _interp_universe_fast skfuzzy):
//  cari indeks tempat (mf>=cut) berubah, interpolasi linear posisinya.
// ----------------------------------------------------------------------------
inline void crossPoints(const std::vector<double>& gx,
                        const std::vector<double>& gmf,
                        double cut, std::vector<double>& out) {
    if (cut <= 0.0) return; // cut=0 hanya menambah titik yg sudah ada di grid
    int n = (int)gx.size();
    for (int i = 0; i < n-1; ++i) {
        bool a = (gmf[i]   >= cut);
        bool b = (gmf[i+1] >= cut);
        if (a != b) {
            double denom = (gmf[i+1] - gmf[i]);
            if (std::fabs(denom) < 1e-12) continue;
            double xx = gx[i] + (cut - gmf[i]) * (gx[i+1] - gx[i]) / denom;
            out.push_back(xx);
        }
    }
}

// ----------------------------------------------------------------------------
//  Defuzzifikasi centroid 3 term, meniru ControlSystem skfuzzy persis.
// ----------------------------------------------------------------------------
inline double defuzzCentroid(double lo, double hi, int n,
                             MF mf0, double cut0,
                             MF mf1, double cut1,
                             MF mf2, double cut2) {
    // --- grid dasar + sampel MF tiap term ---
    //  Universe dibangkitkan PERSIS spt numpy.arange (algoritma DOUBLE_fill):
    //    x[0]=lo ; x[1]=lo+step ; delta=x[1]-lo ; x[i]=lo + i*delta (i>=2).
    //  Ini mereproduksi float-drift numpy: utk gain (lo=0.5,n=201) titik akhir
    //  jadi 2.5000000000000018 (> 2.5) sehingga g_kuat=0 di ujung -- sama dgn
    //  scikit-fuzzy. Utk kecepatan (lo=0,n=101) titik akhir tetap pas 1.0.
    std::vector<double> gx(n);
    std::vector<double> m0(n), m1(n), m2(n);
    double step  = (hi - lo) / (double)(n - 1);   // == 0.01 (double) utk kedua semesta
    double delta = (lo + step) - lo;              // delta ala numpy DOUBLE_fill
    for (int i = 0; i < n; ++i) {
        double x;
        if (i == 0)      x = lo;
        else if (i == 1) x = lo + step;
        else             x = lo + (double)i * delta;
        gx[i] = x;
        m0[i] = mf0(x); m1[i] = mf1(x); m2[i] = mf2(x);
    }

    // --- universe ter-upsample: grid dasar + titik potong tiap term aktif ---
    std::vector<double> uni = gx;
    crossPoints(gx, m0, cut0, uni);
    crossPoints(gx, m1, cut1, uni);
    crossPoints(gx, m2, cut2, uni);
    std::sort(uni.begin(), uni.end());
    uni.erase(std::unique(uni.begin(), uni.end(),
              [](double a, double b){ return std::fabs(a-b) < 1e-12; }), uni.end());

    // --- agregasi (clip min, lalu max) pada tiap titik universe ---
    int M = (int)uni.size();
    std::vector<double> agg(M);
    for (int i = 0; i < M; ++i) {
        double x = uni[i];
        double v0 = std::min(cut0, interpMF(gx, m0, x));
        double v1 = std::min(cut1, interpMF(gx, m1, x));
        double v2 = std::min(cut2, interpMF(gx, m2, x));
        agg[i] = std::max(v0, std::max(v1, v2));
    }

    // --- centroid via integrasi area piecewise-linear EKSAK (persis skfuzzy) ---
    double sum_moment_area = 0.0, sum_area = 0.0;
    for (int i = 1; i < M; ++i) {
        double x1 = uni[i-1], x2 = uni[i];
        double y1 = agg[i-1], y2 = agg[i];
        if ((y1 == 0.0 && y2 == 0.0) || x1 == x2) continue; // luas nol
        double moment, area;
        if (y1 == y2) {                         // persegi panjang
            moment = 0.5 * (x1 + x2);
            area   = (x2 - x1) * y1;
        } else if (y1 == 0.0) {                 // segitiga, tinggi y2
            moment = 2.0/3.0 * (x2 - x1) + x1;
            area   = 0.5 * (x2 - x1) * y2;
        } else if (y2 == 0.0) {                 // segitiga, tinggi y1
            moment = 1.0/3.0 * (x2 - x1) + x1;
            area   = 0.5 * (x2 - x1) * y1;
        } else {                                // trapesium
            moment = (2.0/3.0 * (x2 - x1) * (y2 + 0.5 * y1)) / (y1 + y2) + x1;
            area   = 0.5 * (x2 - x1) * (y1 + y2);
        }
        sum_moment_area += moment * area;
        sum_area        += area;
    }
    if (sum_area <= 1e-12) return 0.5 * (lo + hi); // fallback aman
    return sum_moment_area / sum_area;
}

// ----------------------------------------------------------------------------
//  fuzzyCompute -- API utama.
//    jarak  : jarak rintangan terdekat (m), di-clamp ke [0,2]
//    trav   : traversability thermal [0,1], di-clamp
//  Output (by-ref):
//    kecepatan_scale : [0..1]      (kalikan ke max_vel)
//    gain_belok      : [0.5..2.5]  (pengganti rep_gain crisp)
// ----------------------------------------------------------------------------
inline void fuzzyCompute(double jarak, double trav,
                         double &kecepatan_scale, double &gain_belok)
{
    if (jarak < 0.0) jarak = 0.0; else if (jarak > 2.0) jarak = 2.0;
    if (trav  < 0.0) trav  = 0.0; else if (trav  > 1.0) trav  = 1.0;

    // --- fuzzifikasi input ---
    double jd = j_dekat(jarak),  js = j_sedang(jarak), jj = j_jauh(jarak);
    double tr = t_rendah(trav),  ts = t_sedang(trav),  tt = t_tinggi(trav);

    // --- kekuatan 9 aturan (AND = min) ---
    double r_dr = std::min(jd, tr); // dekat-rendah
    double r_ds = std::min(jd, ts); // dekat-sedang
    double r_dt = std::min(jd, tt); // dekat-tinggi
    double r_sr = std::min(js, tr); // sedang-rendah
    double r_ss = std::min(js, ts); // sedang-sedang
    double r_st = std::min(js, tt); // sedang-tinggi
    double r_jr = std::min(jj, tr); // jauh-rendah
    double r_js = std::min(jj, ts); // jauh-sedang
    double r_jt = std::min(jj, tt); // jauh-tinggi

    // --- agregasi aktivasi per term output (max atas aturan penyumbang) ---
    // Tabel aturan (BAB 3):
    //   kecepatan: Lambat <- {DR,DS,SR}; Sedang <- {DT,SS,JR}; Cepat <- {ST,JS,JT}
    //   gain     : Kuat   <- {DR,DS,SR}; Sedang <- {DT,SS,JR}; Lemah <- {ST,JS,JT}
    double act_kec_lambat = std::max(r_dr, std::max(r_ds, r_sr));
    double act_kec_sedang = std::max(r_dt, std::max(r_ss, r_jr));
    double act_kec_cepat  = std::max(r_st, std::max(r_js, r_jt));

    double act_gain_kuat   = std::max(r_dr, std::max(r_ds, r_sr));
    double act_gain_sedang = std::max(r_dt, std::max(r_ss, r_jr));
    double act_gain_lemah  = std::max(r_st, std::max(r_js, r_jt));

    // --- defuzzifikasi centroid ---
    kecepatan_scale = defuzzCentroid(
        0.0, 1.0, 101,
        k_lambat, act_kec_lambat,
        k_sedang, act_kec_sedang,
        k_cepat,  act_kec_cepat);

    gain_belok = defuzzCentroid(
        0.5, 2.5, 201,
        g_lemah,  act_gain_lemah,
        g_sedang, act_gain_sedang,
        g_kuat,   act_gain_kuat);
}

} // namespace fuzzy

#endif // FUZZY_CONTROLLER_H