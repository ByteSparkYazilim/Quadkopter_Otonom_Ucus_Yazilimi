# Quadkopter_Otonom_Ucus_Yazilimi

##### ByteSpark Robotik firmamızın temellerinin atıldığı “Robotik ve Otomasyon AR-GE Süreci” kapsamında, ekibimiz tarafından, 2020-2022 yılları arasında 3 versiyondan oluşmak üzere “Döner Kanatlı Hava Aracı Otonom Uçuş Kontrol Yazılımı” geliştirilmiştir. Otonom Uçuş Kontrol Yazılımlarının geliştirilme amacı, hali hazırda piyasada bulunan mevut uçuş kontrol yazılımlarının, görece daha basitleştirilmiş ve amaca uygun hale getirilmiş yerli alternatifinin piyasaya sürülmesidir.


##### Otonom Uçuş Yazılımı, ARM tabanlı 32 Bit STM32F407VGT6 mikrodenetleyicisi üzerinde, STM32- CUBE IDE, U-VISION 4: KEIL ve Visual Studio Code ortamlarında geliştirilmiştir. Çalışmalar süresince; STM32 CUBE IDE Debuger, STM Studio, Cube Monitor ve firmamıza ait yer istasyonu yazılımı üzerinden debug, gözlem ve geliştirime işlemleri gerçekleştirilmiştir.

##### Yazılımın geliştirilme süreci için, ekibimiz tarafından 2 ayrı insansız hava aracı test düzeneği geliştirilmiştir. Test düzeneklerinin amacı, insansız hava araçlarının 3 serbestlik dereceli uzayda testlerinin gerçekleştirilebilmesi için güvenli ortam oluşturmak ve güç kaynağına bağlı testler gerçekleştirebilmektir. Geliştirilen bu insansız hava aracı test düzenekleri sayesinde yazılım tepki sonuçları, güvenli ortamda testlere sokularak dış çıktıları gözlemlenmiştir. Geliştirilen test düzeneklerinin 1. Versiyonu, ekibimiz üyesi Hüseyin Can Küçüksezer tarafından makale halinde yayımlanmıştır. 

[Buraya tıklayarak ilgili makalenin tamamına erişebilirsiniz.](https://dergipark.org.tr/tr/download/article-file/1786532)

##### (Gerçekleştirilen testler sırasında; üretilen düzeneklerin mekanik kayıpları, matematiksel analizlere dahil edilmiştir.)


##### 3 Serbestlik dereceli, ahşap malzemeden üretilmiş olan insansız hava aracı test düzeneği yapısı.
https://github.com/ByteSparkYazilim/Quadkopter_Otonom_Ucus_Yazilimi/assets/145047961/53422cb9-074a-4fa8-bd2b-c573f615a97c

##### Ekibimizin üretmiş olduğu HUMA-H1 hegzakopter insansız hava aracının, KK2 uçuş kontrol kartı ile test mekanizması üzerinde kontrol edilmesi.
https://github.com/ByteSparkYazilim/Quadkopter_Otonom_Ucus_Yazilimi/assets/145047961/a922c6d5-d118-451d-b0b6-512e35bbd901



##### Otonom Uçuş Kontrol Yazılımı Üzerinde Bulunan Başlıca Algoritmalar:

* Saniyede 100 İşlem Kapasiteli Denge Kontrol Algoritması
* Saniyede 20 İşlem Kapasiteli İrtifa Kontrol Algoritması
* Saniyede 10 İşlem Kapasiteli Konum Kontrol Algoritması
* S-BUS USART Protokol ve PPM Darbe Kontrollü Evrensel Standartlı RF Haberleşme Yazılımı
* USART Protokolü ile Haberleşme Gerçekleştirebilen Telemetri Yazılımı
* USART Protokolü ile Raspberry Pi Haberleşme ve Kontrol Yazılımı
* Hassas Konum Tespiti için INS ve Yapay Zeka Destekli Konum Hassaslaştırma Algoritması
* Acil Durumlara Karşı; İniş, Motor Kapatma veya Asılı Kalma İşlevlerini Gerçekleştirebilen Fail-
Safe Kontrol Algoritması
* Otonom Kalkış/İniş Algoritması

##### İHA üzerinde bulunan fırçasız DC motorların kontrolü için ESC (Elektronik Hız Kontrolcüsü) modülleri kullanılmıştır. Kullanılan ESC’ler, 50Hz PWM sinyalleri ile kontrol edilmekte olup, doluluk oranlarının değişimine bağlı olarak çıkış fazlarına düşen I/O hızlarını değiştirmektedir.

## Motor Sürme Fonksiyonu:

```python
void motor_sur(int motor1, int motor2, int motor3, int motor4, bool arm_check_val){

if(arm_check_val==false){

HAL_TIM_PWM_Stop(&amp;htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Stop(&amp;htim3,TIM_CHANNEL_2);
HAL_TIM_PWM_Stop(&amp;htim3,TIM_CHANNEL_3);
HAL_TIM_PWM_Stop(&amp;htim3,TIM_CHANNEL_4);

}

if(arm_check_val==true){
__HAL_TIM_SET_COMPARE(&amp;htim3,TIM_CHANNEL_1,motor3);// 3 numara
__HAL_TIM_SET_COMPARE(&amp;htim3,TIM_CHANNEL_2,motor4);// 4 numara

__HAL_TIM_SET_COMPARE(&amp;htim3,TIM_CHANNEL_3,motor1);// 1 numara
__HAL_TIM_SET_COMPARE(&amp;htim3,TIM_CHANNEL_4,motor2);// 2 numara
}

}
```

##### Yazılımda bulunan ARM_KONTROL değişkeni ile, motorlara güç verilip verilmeyeceği belirlenmektedir. Arm kontrol fonksiyonu, insansız hava aracının güvenli kullanımı için oldukça önemli bir unsurdur.

## Arm Kontrol Fonksiyonu:
```python
int huma_arm_kontrol(int arm_switch){

if(arm_fonksiyon_kontrol==false){

roll_offset=HUMA_Pitch_Get();
pitch_offset=HUMA_Roll_Get();

if(arm_switch&gt;1600 &amp;&amp; throttle_map&lt;1050){
HAL_TIM_Base_Start_IT(&amp;htim8);
if (KumandaSayac&gt;arm_suresi){

HAL_GPIO_WritePin(GPIOD,led_arm_Pin,GPIO_PIN_SET);
//HAL_Delay(2000);
HAL_TIM_PWM_Start(&amp;htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&amp;htim3,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&amp;htim3,TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&amp;htim3,TIM_CHANNEL_4);

arm=true;
arm_fonksiyon_kontrol=true;

HAL_TIM_Base_Stop_IT(&amp;htim8);
KumandaSayac=0;

}

}
else {

HAL_TIM_PWM_Stop(&amp;htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Stop(&amp;htim3,TIM_CHANNEL_2);
HAL_TIM_PWM_Stop(&amp;htim3,TIM_CHANNEL_3);
HAL_TIM_PWM_Stop(&amp;htim3,TIM_CHANNEL_4);
HAL_TIM_Base_Stop(&amp;htim8);
KumandaSayac=0;

}

}
```

##### Arm Kontrol fonksiyonu, RC kumanda üzerinden yapılan belirli komut hareketlerine göre bir sayaç sayarak, belirli süre sonunda motorları sürecek timer kesmesinin aktifleştirilmesini sağlamaktır. Belirlenen süre boyunca RC kumandadan ARM komutu gönderilmediği takdirde, motorları sürecek timer fonksiyonu pasif durumda kalmaktadır. Ayrıca DİSARM durumda, insansız hava aracının bulunduğu yüzeye göre kendi denge kalibrasyonunu gerçekleştirmesi, bu fonksiyon içerisinde sağlanmaktadır.

##### İnsansız hava aracının motorlarına gönderilecek PWM sinyallerinin belirlenmesi için PID kontrol gerçekleştirilmektedir. Kontrol algoritmasından çıkan değerlerin motorlara gönderilmeden önce hesaplanarak toparlanması, Motor_Güç_Değeri fonksiyonları aracılığıyla sağlanmaktadır.

## Motor Güç Değeri Fonksiyonu:
```python
int motor1_guc_degeri(int Roll_output_pid, int Pitch_output_pid, int Yaw_output_pid, int
throttle_kumanda_degeri){

motor1_guc_cikis = throttle_kumanda_degeri + Yaw_output_pid + Pitch_output_pid +
Roll_output_pid;

if(motor1_guc_cikis &gt; 1900) {motor1_guc_cikis = 1900;}
if(motor1_guc_cikis &lt; 1200) {motor1_guc_cikis = 1200;}
if(throttle_kumanda_degeri&lt;1200) motor1_guc_cikis=throttle_kumanda_degeri;

return motor1_guc_cikis;
}
```

##### Kontrol algoritmasından gelen matematiksel sonuçların, ESC’lerin sürüleceği frekansa ait doluluk oranı sınırlarının içerisinde kalması için, çıktılara sınırlama getirilmiştir.

##### RC kumanda üzerinden okunan ve YAW eksenini kontrol eden kontrol çubuğundan gelen verilere,yazılımsal olarak ölü bölge tanımlanmıştır. Ölü bölge tanımı yapılmasının sebebi, insansız hava aracının YAW ekseninde 3-5 derece arasında normal kabul edilebilir sapmaları olmasıdır. (Kontrolalgoritması kullanılmadığı durumlarda) Bu sapmalar, PID kontrol algoritmasında çok yüksek D-kick durumları ortaya çıkartmaktadır. Bu sebeple RC kumanda ve Manyetometre sensörü arasında oluşan hassas farklar, matematiksel kontrol yöntemlerine dahil edilmemiştir. Ölü bölgenin hazırlanma sebebi, bu değerlerin devreden çıkartılmasıdır.

##### Uçuş Kontrol Kartı çevresel birimlerinden alınan verilere Kalman filtresi uygulanarak parazitlerin giderilmesi ve hassasiyetin artırılması sağlanmıştır.

## BME280 Barometrik Sensör İçin Kalman Filtresi Uygulaması:
```python
#include &quot;KalmanBME280.h&quot;
#include &quot;math.h&quot;

#define KF_Z_MEAS_VARIANCE 150

#define KF_A_MEAS_VARIANCE 50.0f
#define KF_ACCELBIAS_VARIANCE 0.005f

float Pzz,Pzv,Pza,Pzb;
float Pvz,Pvv,Pva,Pvb;
float Paz,Pav,Paa,Pab;
float Pbz,Pbv,Pba,Pbb;

float AccelVariance;
float ABiasVariance;
float ZMeasVariance;
float AMeasVariance;
float KAdapt;

float z;
float v;
float a;
float b;

void kalmanFilter4d_configure(float aVariance, float kAdapt, float zInitial, float vInitial, float aInitial){
ZMeasVariance = KF_Z_MEAS_VARIANCE;
AMeasVariance = KF_A_MEAS_VARIANCE;
ABiasVariance = KF_ACCELBIAS_VARIANCE;
AccelVariance = 120;

KAdapt = kAdapt;

z = zInitial;
v = vInitial;
a = aInitial;

b = 0.0f;

Pzz = 1500.0f; //Pzz = 1500.0f;
Pzv = 0.0f; //Pzv = 0.0f;
Pza = 0.0f; //Pza = 0.0f;
Pzb = 0.0f; //Pzb = 0.0f;

Pvz = Pzv;
Pvv = 1500.0f;
Pva = 0.0f;
Pvb = 0.0f;

Paz = Pza;
Pav = Pva;
Paa = 100000.0f;//Paa = 100000.0f;
Pab = 0.0f;

Pbz = Pzb;
Pbv = Pvb;
Pba = Pab;
Pbb = 1500.0f;
}

void kalmanFilter4d_predict(float dt) {

z = z + (v * dt) + (accel_true * dt * dt* 0.5f);
v = v + (accel_true * dt);

// Predicted (a priori) state covariance estimate P_k- = (F * P_k-1+ * F_t) + Qk
float dt2 = dt*dt; // dt^2
float dt3 = dt2*dt; // dt^3

float dt4 = dt2*dt2; // dt^4;
float dt2div2 = dt2*0.5f; // dt^2/2
float dt3div2 = dt3*0.5f; // dt^3/2
float dt4div2 = dt4*0.5f; // dt^4/2
float dt4div4 = dt4*0.25f; // dt^4/4

float p00 = Pzz + 2.0f*Pzv*dt + (Pza - Pzb)*dt2 + Pvv*dt2div2 + (Pva - Pvb)*dt3 +
(Paa+Pbb)*dt4div4 - Pab*dt4div2;
float p01 = Pzv + dt*(Pza - Pzb + Pvv) + 3.0f*dt2div2*(Pva - Pvb) - Pab*dt3 + (Paa +
Pbb)*dt3div2;
float p02 = Pza + Pva*dt + (Paa - Pba)*dt2div2;
float p03 = Pzb + Pvb*dt + (Pab - Pbb)*dt2div2;

float p11 = Pvv + 2.0f*dt*(Pva - Pvb) + dt2*(Paa - 2.0f*Pab + Pbb);
float p12 = Pva + dt*(Paa - Pba);
float p13 = Pvb + dt*(Pab - Pbb);

float p22 = Paa;
float p23 = Pab;
float p33 = Pbb;

Pzz = p00;
Pzv = p01;
Pza = p02;
Pzb = p03;

Pvz = Pzv;
Pvv = p11;
Pva = p12;
Pvb = p13;

Paz = Pza;

Pav = Pva;
Paa = p22;
Pab = p23;

Pbz = Pzb;
Pbv = Pvb;
Pba = Pab;
Pbb = p33;

Paa = Paa + AccelVariance;
Pbb = Pbb + ABiasVariance;
}

float z_err;
float a_err;

float kalmanFilter4d_update(float zm, float am, float* pz, float* pv) {

z_err = zm - z;
a_err = am - a;

// S_k = (H * P_k- * H_t) + R_k
float s00 = Pzz;
float s01 = Pza;
float s10 = s01;
float s11 = Paa;

s00 += ZMeasVariance;
s11 += AMeasVariance;

float accel_ext = fabs(am-b);// abs() --&gt; fabs()
s11 += (KAdapt*accel_ext*accel_ext);

ABiasVariance = KF_ACCELBIAS_VARIANCE/(1.0f + accel_ext);

float sdetinv = 1.0f/(s00*s11 - s10*s01);
float sinv00 = sdetinv * s11;
float sinv01 = -sdetinv * s10;
float sinv10 = sinv01;
float sinv11 = sdetinv * s00;

float k00 = Pzz*sinv00 + Pza*sinv10;
float k01 = Pzz*sinv01 + Pza*sinv11;
float k10 = Pvz*sinv00 + Pva*sinv10;
float k11 = Pvz*sinv01 + Pva*sinv11;
float k20 = Paz*sinv00 + Paa*sinv10;
float k21 = Paz*sinv01 + Paa*sinv11;
float k30 = Pbz*sinv00 + Pba*sinv10;
float k31 = Pbz*sinv01 + Pba*sinv11;

z = z + (k00*z_err + k01*a_err);
v = v + (k10*z_err + k11*a_err);
a = a + (k20*z_err + k21*a_err);
b = b + (k30*z_err + k31*a_err);

float tmp = 1.0f - k00;

float p00 = tmp*Pzz - k01*Paz;
float p01 = tmp*Pzv - k01*Pav;
float p02 = tmp*Pza - k01*Paa;
float p03 = tmp*Pzb - k01*Pab;

float p11 = -k10*Pzv + Pvv - k11*Pav;
float p12 = -k10*Pza + Pva - k11*Paa;
float p13 = -k10*Pzb + Pvb - k11*Pab;

float p22 = -k20*Pza + (1.0f-k21)*Paa;
float p23 = -k20*Pzb + (1.0f-k21)*Pab;

float p33 = -k30*Pzb -k31*Pab + Pbb;

Pzz = p00;
Pzv = p01;
Pza = p02;
Pzb = p03;

Pvz = Pzv;
Pvv = p11;
Pva = p12;
Pvb = p13;

Paz = Pza;
Pav = Pva;
Paa = p22;
Pab = p23;

Pbz = Pzb;
Pbv = Pvb;

Pba = Pab;
Pbb = p33;

*pz = z;
*pv = v;
return z;
}

```

##### PID algoritması; temel olarak RC kumanda ve üzerinde yer alan 3 serbestlik dereceli +- 9DOF IMU sensörünün verilerinin kıyaslanması ile çalıştırılmaktadır. RC kumandadan gelen yönelim değerleri ile IMU verileri kıyaslanarak, insansız hava aracının olması gereken konum belirlenir. RC kumandaya müdahale edilmediği takdirde gelen veriler, insansız hava aracının asılı kalacağı değerler olarak belirlenmiş; +-15 derece yönelim açısı alacak şekilde optimize edilmiştir. YAW eksenindeki hareketin kontrolü için ise ivme kontrolü yapılarak PID algoritması hazırlanmıştır. Bu noktada kullanılan ölü bölge sayesinde, yüksek ivmeli değerler dışında tepki verilmemesi sağlanarak stabil uçuş elde edilmiştir. İrtifa kontrolü için hazırlanan sistem, BME280 barometrik sensöründen alınan irtifa verilerine göre hesaplanmakta olup, PD kontrol yöntemiyle hazırlanmıştır.

#### Uçuş Kontrol Yazılımında Bulunan Çevresel Bağlantı Elemanları ve Protokolleri:
##### IMU (Inertial Measurment Unit): I2C-100Hz-DMA Interrupt
##### BME280 Barometrik Sensör: I2C-100Hz-DMA Interrupt
##### GPS (Global Position System): UART – 10Hz- DMA
##### RC Kumanda: USART-100Hz-DMA / PPM-100Hz-Input Capture w/DMA
##### 2.4GHz Telemetri: USART-100Hz-DMA
##### SD Kart Yazıcı/Okuyucu: SPI-1Hz Timer Interrupt-Normal Mod
##### Debug and Serial Connection: Serial Wire 4 Pin / JTAG 5 Pin

##### Bu makalede, uçuş kontrol yazılımımızın bir kısmı görüntülenebilmektedir. PID ve PD kontrolalgoritmaları, Kalman Filtre Algoritmalarının bir kısmı, Hassas Konumlandırma Algoritmaları, Otonom iniş-kalkış algoritmaları vb. algoritmalar veri gizliliği sebebiyle bu makalede paylaşılmamıştır ve/veya detaylı bilgi verilmemiştir. Bilgi edinmek için iletişime geçiniz.
