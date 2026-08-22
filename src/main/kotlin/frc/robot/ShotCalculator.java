package frc.robot;

public class ShotCalculator {

public static double calculateVelocity(double distance, double vx, double vy) {
return
        5.97853660 +
        (-0.66388701) * distance +
        (-0.09874967) * vx +
        (-0.00086549) * vy +
        (0.71642555) * Math.pow(distance, 2.0) +
        (-0.21847538) * distance*vx +
        (0.00009452) * distance*vy +
        (0.00146370) * Math.pow(vx, 2.0) +
        (-0.00415667) * vx*vy +
        (0.12223330) * Math.pow(vy, 2.0) +
        (-0.14298260) * Math.pow(distance, 3.0) +
        (0.07437115) * Math.pow(distance, 2.0)*vx +
        (0.00007023) * Math.pow(distance, 2.0)*vy +
        (0.02589207) * distance*Math.pow(vx, 2.0) +
        (0.00193282) * distance*vx*vy +
        (-0.03277615) * distance*Math.pow(vy, 2.0) +
        (0.00612054) * Math.pow(vx, 3.0) +
        (-0.00038519) * Math.pow(vx, 2.0)*vy +
        (0.01505595) * vx*Math.pow(vy, 2.0) +
        (0.00051517) * Math.pow(vy, 3.0) +
        (0.00925977) * Math.pow(distance, 4.0) +
        (-0.00985721) * Math.pow(distance, 3.0)*vx +
        (-0.00000976) * Math.pow(distance, 3.0)*vy +
        (-0.00280209) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0) +
        (-0.00025822) * Math.pow(distance, 2.0)*vx*vy +
        (0.00426756) * Math.pow(distance, 2.0)*Math.pow(vy, 2.0) +
        (-0.00291321) * distance*Math.pow(vx, 3.0) +
        (0.00008520) * distance*Math.pow(vx, 2.0)*vy +
        (-0.00443858) * distance*vx*Math.pow(vy, 2.0) +
        (-0.00011814) * distance*Math.pow(vy, 3.0) +
        (0.00446003) * Math.pow(vx, 4.0) +
        (0.00024461) * Math.pow(vx, 3.0)*vy +
        (0.00322429) * Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (0.00021100) * vx*Math.pow(vy, 3.0) +
        (0.00048554) * Math.pow(vy, 4.0)
        ;
}




public static double calculatePitch(double distance, double vx, double vy) {
return
        78.49182897 +
        (0.03347185) * distance +
        (2.02900438) * vx +
        (0.00457273) * vy +
        (-0.31250198) * Math.pow(distance, 2.0) +
        (2.57000848) * distance*vx +
        (-0.01548941) * distance*vy +
        (-1.19374583) * Math.pow(vx, 2.0) +
        (0.02325538) * vx*vy +
        (-1.71825184) * Math.pow(vy, 2.0) +
        (-0.79748121) * Math.pow(distance, 3.0) +
        (2.02845574) * Math.pow(distance, 2.0)*vx +
        (-0.01073922) * Math.pow(distance, 2.0)*vy +
        (-0.88451480) * distance*Math.pow(vx, 2.0) +
        (-0.00592655) * distance*vx*vy +
        (-0.03142625) * distance*Math.pow(vy, 2.0) +
        (-0.17233626) * Math.pow(vx, 3.0) +
        (0.00097022) * Math.pow(vx, 2.0)*vy +
        (0.78703065) * vx*Math.pow(vy, 2.0) +
        (0.07268858) * Math.pow(vy, 3.0) +
        (-0.80133064) * Math.pow(distance, 4.0) +
        (0.17218795) * Math.pow(distance, 3.0)*vx +
        (0.00294997) * Math.pow(distance, 3.0)*vy +
        (-0.73736715) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0) +
        (-0.02375141) * Math.pow(distance, 2.0)*vx*vy +
        (0.39592100) * Math.pow(distance, 2.0)*Math.pow(vy, 2.0) +
        (-0.10729864) * distance*Math.pow(vx, 3.0) +
        (-0.02035681) * distance*Math.pow(vx, 2.0)*vy +
        (0.68502714) * distance*vx*Math.pow(vy, 2.0) +
        (0.00921807) * distance*Math.pow(vy, 3.0) +
        (-0.81545347) * Math.pow(vx, 4.0) +
        (0.00733194) * Math.pow(vx, 3.0)*vy +
        (-0.01993230) * Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (0.02819437) * vx*Math.pow(vy, 3.0) +
        (-1.79612265) * Math.pow(vy, 4.0) +
        (0.04453862) * Math.pow(distance, 5.0) +
        (-1.64156850) * Math.pow(distance, 4.0)*vx +
        (0.00170397) * Math.pow(distance, 4.0)*vy +
        (-0.42258754) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0) +
        (-0.01353571) * Math.pow(distance, 3.0)*vx*vy +
        (-0.20231247) * Math.pow(distance, 3.0)*Math.pow(vy, 2.0) +
        (-0.46266358) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0) +
        (-0.00982907) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*vy +
        (0.15080180) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 2.0) +
        (0.01870287) * Math.pow(distance, 2.0)*Math.pow(vy, 3.0) +
        (0.10532430) * distance*Math.pow(vx, 4.0) +
        (-0.02389940) * distance*Math.pow(vx, 3.0)*vy +
        (1.48322769) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.01072090) * distance*vx*Math.pow(vy, 3.0) +
        (1.18842100) * distance*Math.pow(vy, 4.0) +
        (-0.40627475) * Math.pow(vx, 5.0) +
        (-0.00044083) * Math.pow(vx, 4.0)*vy +
        (-0.12280417) * Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (0.00036074) * Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.00748665) * vx*Math.pow(vy, 4.0) +
        (-0.01142804) * Math.pow(vy, 5.0) +
        (0.68166506) * Math.pow(distance, 6.0) +
        (-0.67216640) * Math.pow(distance, 5.0)*vx +
        (-0.00566017) * Math.pow(distance, 5.0)*vy +
        (0.31527146) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0) +
        (0.01569048) * Math.pow(distance, 4.0)*vx*vy +
        (-0.38915804) * Math.pow(distance, 4.0)*Math.pow(vy, 2.0) +
        (-0.84832194) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0) +
        (-0.00021506) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*vy +
        (-0.44968171) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 2.0) +
        (-0.00244429) * Math.pow(distance, 3.0)*Math.pow(vy, 3.0) +
        (-0.02853924) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0) +
        (-0.05050075) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*vy +
        (0.90943696) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.02706893) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 3.0) +
        (0.41005504) * Math.pow(distance, 2.0)*Math.pow(vy, 4.0) +
        (-0.11063155) * distance*Math.pow(vx, 5.0) +
        (-0.02841401) * distance*Math.pow(vx, 4.0)*vy +
        (0.20596533) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (0.00146590) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.08286613) * distance*vx*Math.pow(vy, 4.0) +
        (-0.05190545) * distance*Math.pow(vy, 5.0) +
        (-0.15810292) * Math.pow(vx, 6.0) +
        (0.05594023) * Math.pow(vx, 5.0)*vy +
        (-0.51072291) * Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (0.01237516) * Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (-0.28094508) * Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (0.07552108) * vx*Math.pow(vy, 5.0) +
        (-0.85273550) * Math.pow(vy, 6.0) +
        (-0.30636204) * Math.pow(distance, 7.0) +
        (2.07856563) * Math.pow(distance, 6.0)*vx +
        (0.01327680) * Math.pow(distance, 6.0)*vy +
        (0.68088795) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0) +
        (0.01669876) * Math.pow(distance, 5.0)*vx*vy +
        (0.45402332) * Math.pow(distance, 5.0)*Math.pow(vy, 2.0) +
        (-0.25348816) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0) +
        (-0.00628211) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*vy +
        (-0.35258482) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 2.0) +
        (-0.05660423) * Math.pow(distance, 4.0)*Math.pow(vy, 3.0) +
        (-0.39077668) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0) +
        (-0.01795220) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*vy +
        (-0.63774647) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.00951619) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 3.0) +
        (-1.02521582) * Math.pow(distance, 3.0)*Math.pow(vy, 4.0) +
        (0.27907954) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0) +
        (0.00603582) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*vy +
        (0.30120364) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (0.02157439) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.14257275) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 4.0) +
        (0.07706979) * Math.pow(distance, 2.0)*Math.pow(vy, 5.0) +
        (1.14338455) * distance*Math.pow(vx, 6.0) +
        (0.02454351) * distance*Math.pow(vx, 5.0)*vy +
        (0.48666245) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (0.01279868) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (1.32216597) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (0.05103562) * distance*vx*Math.pow(vy, 5.0) +
        (2.01259772) * distance*Math.pow(vy, 6.0) +
        (0.05098016) * Math.pow(vx, 7.0) +
        (0.02528207) * Math.pow(vx, 6.0)*vy +
        (-0.17538127) * Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (-0.03346430) * Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (0.50162717) * Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (0.03577164) * Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (-1.05021260) * vx*Math.pow(vy, 6.0) +
        (-0.08617724) * Math.pow(vy, 7.0) +
        (-0.01151109) * Math.pow(distance, 8.0) +
        (-1.33955812) * Math.pow(distance, 7.0)*vx +
        (-0.01051310) * Math.pow(distance, 7.0)*vy +
        (-0.49648363) * Math.pow(distance, 6.0)*Math.pow(vx, 2.0) +
        (-0.01996719) * Math.pow(distance, 6.0)*vx*vy +
        (-0.20102685) * Math.pow(distance, 6.0)*Math.pow(vy, 2.0) +
        (0.96692098) * Math.pow(distance, 5.0)*Math.pow(vx, 3.0) +
        (0.01465375) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0)*vy +
        (0.31240163) * Math.pow(distance, 5.0)*vx*Math.pow(vy, 2.0) +
        (0.05372077) * Math.pow(distance, 5.0)*Math.pow(vy, 3.0) +
        (0.04887129) * Math.pow(distance, 4.0)*Math.pow(vx, 4.0) +
        (0.05383951) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0)*vy +
        (-0.62925678) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (0.01471230) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 3.0) +
        (0.71109004) * Math.pow(distance, 4.0)*Math.pow(vy, 4.0) +
        (0.34083756) * Math.pow(distance, 3.0)*Math.pow(vx, 5.0) +
        (0.00932413) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0)*vy +
        (0.05574681) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (-0.04048944) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.20625846) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 4.0) +
        (-0.06638588) * Math.pow(distance, 3.0)*Math.pow(vy, 5.0) +
        (-0.10485038) * Math.pow(distance, 2.0)*Math.pow(vx, 6.0) +
        (-0.03750898) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0)*vy +
        (-0.53127603) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (0.01571274) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (-0.47346269) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (0.00317750) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 5.0) +
        (-1.69052604) * Math.pow(distance, 2.0)*Math.pow(vy, 6.0) +
        (-0.10717247) * distance*Math.pow(vx, 7.0) +
        (-0.02251882) * distance*Math.pow(vx, 6.0)*vy +
        (0.28272737) * distance*Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (0.04017652) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (0.41748173) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (0.11939507) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (-0.25176255) * distance*vx*Math.pow(vy, 6.0) +
        (0.08539351) * distance*Math.pow(vy, 7.0) +
        (-0.38531812) * Math.pow(vx, 8.0) +
        (-0.08176471) * Math.pow(vx, 7.0)*vy +
        (0.71633162) * Math.pow(vx, 6.0)*Math.pow(vy, 2.0) +
        (0.07085649) * Math.pow(vx, 5.0)*Math.pow(vy, 3.0) +
        (-0.18493713) * Math.pow(vx, 4.0)*Math.pow(vy, 4.0) +
        (-0.03951611) * Math.pow(vx, 3.0)*Math.pow(vy, 5.0) +
        (-0.48044027) * Math.pow(vx, 2.0)*Math.pow(vy, 6.0) +
        (-0.10989280) * vx*Math.pow(vy, 7.0) +
        (0.15568826) * Math.pow(vy, 8.0) +
        (0.03760161) * Math.pow(distance, 9.0) +
        (0.42152198) * Math.pow(distance, 8.0)*vx +
        (0.00391834) * Math.pow(distance, 8.0)*vy +
        (0.11566116) * Math.pow(distance, 7.0)*Math.pow(vx, 2.0) +
        (0.00798146) * Math.pow(distance, 7.0)*vx*vy +
        (0.04843285) * Math.pow(distance, 7.0)*Math.pow(vy, 2.0) +
        (-0.49362937) * Math.pow(distance, 6.0)*Math.pow(vx, 3.0) +
        (-0.00804367) * Math.pow(distance, 6.0)*Math.pow(vx, 2.0)*vy +
        (-0.05809280) * Math.pow(distance, 6.0)*vx*Math.pow(vy, 2.0) +
        (-0.02133506) * Math.pow(distance, 6.0)*Math.pow(vy, 3.0) +
        (0.12428083) * Math.pow(distance, 5.0)*Math.pow(vx, 4.0) +
        (-0.02713843) * Math.pow(distance, 5.0)*Math.pow(vx, 3.0)*vy +
        (0.50738232) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.00403784) * Math.pow(distance, 5.0)*vx*Math.pow(vy, 3.0) +
        (-0.27398128) * Math.pow(distance, 5.0)*Math.pow(vy, 4.0) +
        (-0.38691003) * Math.pow(distance, 4.0)*Math.pow(vx, 5.0) +
        (-0.00612668) * Math.pow(distance, 4.0)*Math.pow(vx, 4.0)*vy +
        (-0.20267425) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (0.02016927) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (-0.24068038) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 4.0) +
        (0.02888403) * Math.pow(distance, 4.0)*Math.pow(vy, 5.0) +
        (-0.22620762) * Math.pow(distance, 3.0)*Math.pow(vx, 6.0) +
        (0.01969774) * Math.pow(distance, 3.0)*Math.pow(vx, 5.0)*vy +
        (0.35505830) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (-0.02100364) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (0.04744508) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (-0.02671966) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 5.0) +
        (0.76940842) * Math.pow(distance, 3.0)*Math.pow(vy, 6.0) +
        (0.17669714) * Math.pow(distance, 2.0)*Math.pow(vx, 7.0) +
        (0.00742315) * Math.pow(distance, 2.0)*Math.pow(vx, 6.0)*vy +
        (-0.39701198) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (-0.01169761) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (-0.28518335) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (-0.08620018) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (0.44173503) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 6.0) +
        (-0.02300570) * Math.pow(distance, 2.0)*Math.pow(vy, 7.0) +
        (-0.11297326) * distance*Math.pow(vx, 8.0) +
        (0.04684841) * distance*Math.pow(vx, 7.0)*vy +
        (-0.88819084) * distance*Math.pow(vx, 6.0)*Math.pow(vy, 2.0) +
        (-0.08513832) * distance*Math.pow(vx, 5.0)*Math.pow(vy, 3.0) +
        (0.61966911) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 4.0) +
        (0.08164999) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 5.0) +
        (-0.53121547) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 6.0) +
        (-0.00574776) * distance*vx*Math.pow(vy, 7.0) +
        (-0.19245073) * distance*Math.pow(vy, 8.0) +
        (-0.05123102) * Math.pow(vx, 9.0) +
        (-0.02475578) * Math.pow(vx, 8.0)*vy +
        (0.16666958) * Math.pow(vx, 7.0)*Math.pow(vy, 2.0) +
        (0.04508324) * Math.pow(vx, 6.0)*Math.pow(vy, 3.0) +
        (-0.03765097) * Math.pow(vx, 5.0)*Math.pow(vy, 4.0) +
        (-0.01831649) * Math.pow(vx, 4.0)*Math.pow(vy, 5.0) +
        (-0.35909051) * Math.pow(vx, 3.0)*Math.pow(vy, 6.0) +
        (-0.06250022) * Math.pow(vx, 2.0)*Math.pow(vy, 7.0) +
        (0.49661763) * vx*Math.pow(vy, 8.0) +
        (0.02875468) * Math.pow(vy, 9.0) +
        (-0.01018761) * Math.pow(distance, 10.0) +
        (-0.07194186) * Math.pow(distance, 9.0)*vx +
        (-0.00076465) * Math.pow(distance, 9.0)*vy +
        (-0.00786560) * Math.pow(distance, 8.0)*Math.pow(vx, 2.0) +
        (-0.00157580) * Math.pow(distance, 8.0)*vx*vy +
        (-0.00703719) * Math.pow(distance, 8.0)*Math.pow(vy, 2.0) +
        (0.10826646) * Math.pow(distance, 7.0)*Math.pow(vx, 3.0) +
        (0.00193604) * Math.pow(distance, 7.0)*Math.pow(vx, 2.0)*vy +
        (-0.00341241) * Math.pow(distance, 7.0)*vx*Math.pow(vy, 2.0) +
        (0.00434481) * Math.pow(distance, 7.0)*Math.pow(vy, 3.0) +
        (-0.05183184) * Math.pow(distance, 6.0)*Math.pow(vx, 4.0) +
        (0.00613710) * Math.pow(distance, 6.0)*Math.pow(vx, 3.0)*vy +
        (-0.13399271) * Math.pow(distance, 6.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (0.00040795) * Math.pow(distance, 6.0)*vx*Math.pow(vy, 3.0) +
        (0.06062872) * Math.pow(distance, 6.0)*Math.pow(vy, 4.0) +
        (0.13252216) * Math.pow(distance, 5.0)*Math.pow(vx, 5.0) +
        (0.00153763) * Math.pow(distance, 5.0)*Math.pow(vx, 4.0)*vy +
        (0.07593840) * Math.pow(distance, 5.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (-0.00476253) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.07963969) * Math.pow(distance, 5.0)*vx*Math.pow(vy, 4.0) +
        (-0.00652126) * Math.pow(distance, 5.0)*Math.pow(vy, 5.0) +
        (0.06768892) * Math.pow(distance, 4.0)*Math.pow(vx, 6.0) +
        (-0.00534176) * Math.pow(distance, 4.0)*Math.pow(vx, 5.0)*vy +
        (-0.11282177) * Math.pow(distance, 4.0)*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (0.00729033) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (-0.00992150) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (0.00826975) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 5.0) +
        (-0.18976825) * Math.pow(distance, 4.0)*Math.pow(vy, 6.0) +
        (-0.08794537) * Math.pow(distance, 3.0)*Math.pow(vx, 7.0) +
        (-0.00067141) * Math.pow(distance, 3.0)*Math.pow(vx, 6.0)*vy +
        (0.19600482) * Math.pow(distance, 3.0)*Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (0.00067188) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (0.05609949) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (0.02775245) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (-0.12521924) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 6.0) +
        (0.00202410) * Math.pow(distance, 3.0)*Math.pow(vy, 7.0) +
        (0.15449796) * Math.pow(distance, 2.0)*Math.pow(vx, 8.0) +
        (-0.01497419) * Math.pow(distance, 2.0)*Math.pow(vx, 7.0)*vy +
        (0.36823037) * Math.pow(distance, 2.0)*Math.pow(vx, 6.0)*Math.pow(vy, 2.0) +
        (0.03584399) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0)*Math.pow(vy, 3.0) +
        (-0.30317075) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*Math.pow(vy, 4.0) +
        (-0.03960345) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 5.0) +
        (0.37210168) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 6.0) +
        (0.03013754) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 7.0) +
        (0.05142381) * Math.pow(distance, 2.0)*Math.pow(vy, 8.0) +
        (0.01333569) * distance*Math.pow(vx, 9.0) +
        (0.01531899) * distance*Math.pow(vx, 8.0)*vy +
        (-0.19676333) * distance*Math.pow(vx, 7.0)*Math.pow(vy, 2.0) +
        (-0.03544965) * distance*Math.pow(vx, 6.0)*Math.pow(vy, 3.0) +
        (0.09827402) * distance*Math.pow(vx, 5.0)*Math.pow(vy, 4.0) +
        (0.01726731) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 5.0) +
        (0.02310677) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 6.0) +
        (0.00145986) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 7.0) +
        (-0.20407645) * distance*vx*Math.pow(vy, 8.0) +
        (-0.02503038) * distance*Math.pow(vy, 9.0) +
        (0.09072684) * Math.pow(vx, 10.0) +
        (0.00842378) * Math.pow(vx, 9.0)*vy +
        (-0.02638598) * Math.pow(vx, 8.0)*Math.pow(vy, 2.0) +
        (0.00732371) * Math.pow(vx, 7.0)*Math.pow(vy, 3.0) +
        (0.01007217) * Math.pow(vx, 6.0)*Math.pow(vy, 4.0) +
        (-0.00286003) * Math.pow(vx, 5.0)*Math.pow(vy, 5.0) +
        (-0.07342958) * Math.pow(vx, 4.0)*Math.pow(vy, 6.0) +
        (-0.01658129) * Math.pow(vx, 3.0)*Math.pow(vy, 7.0) +
        (0.20570221) * Math.pow(vx, 2.0)*Math.pow(vy, 8.0) +
        (0.03765447) * vx*Math.pow(vy, 9.0) +
        (-0.00392607) * Math.pow(vy, 10.0) +
        (0.00115176) * Math.pow(distance, 11.0) +
        (0.00638680) * Math.pow(distance, 10.0)*vx +
        (0.00007600) * Math.pow(distance, 10.0)*vy +
        (-0.00060413) * Math.pow(distance, 9.0)*Math.pow(vx, 2.0) +
        (0.00015612) * Math.pow(distance, 9.0)*vx*vy +
        (0.00060509) * Math.pow(distance, 9.0)*Math.pow(vy, 2.0) +
        (-0.01115956) * Math.pow(distance, 8.0)*Math.pow(vx, 3.0) +
        (-0.00022119) * Math.pow(distance, 8.0)*Math.pow(vx, 2.0)*vy +
        (0.00185898) * Math.pow(distance, 8.0)*vx*Math.pow(vy, 2.0) +
        (-0.00045186) * Math.pow(distance, 8.0)*Math.pow(vy, 3.0) +
        (0.00746618) * Math.pow(distance, 7.0)*Math.pow(vx, 4.0) +
        (-0.00066996) * Math.pow(distance, 7.0)*Math.pow(vx, 3.0)*vy +
        (0.01555500) * Math.pow(distance, 7.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.00000944) * Math.pow(distance, 7.0)*vx*Math.pow(vy, 3.0) +
        (-0.00708557) * Math.pow(distance, 7.0)*Math.pow(vy, 4.0) +
        (-0.01893856) * Math.pow(distance, 6.0)*Math.pow(vx, 5.0) +
        (-0.00016219) * Math.pow(distance, 6.0)*Math.pow(vx, 4.0)*vy +
        (-0.01060457) * Math.pow(distance, 6.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (0.00060276) * Math.pow(distance, 6.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (-0.01089263) * Math.pow(distance, 6.0)*vx*Math.pow(vy, 4.0) +
        (0.00080958) * Math.pow(distance, 6.0)*Math.pow(vy, 5.0) +
        (-0.00501029) * Math.pow(distance, 5.0)*Math.pow(vx, 6.0) +
        (0.00071495) * Math.pow(distance, 5.0)*Math.pow(vx, 5.0)*vy +
        (0.01638662) * Math.pow(distance, 5.0)*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (-0.00108320) * Math.pow(distance, 5.0)*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (0.00356919) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (-0.00092042) * Math.pow(distance, 5.0)*vx*Math.pow(vy, 5.0) +
        (0.02388113) * Math.pow(distance, 5.0)*Math.pow(vy, 6.0) +
        (0.01362902) * Math.pow(distance, 4.0)*Math.pow(vx, 7.0) +
        (-0.00015743) * Math.pow(distance, 4.0)*Math.pow(vx, 6.0)*vy +
        (-0.03918108) * Math.pow(distance, 4.0)*Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (0.00000184) * Math.pow(distance, 4.0)*Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (-0.00462128) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (-0.00471761) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (0.01315086) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 6.0) +
        (-0.00066092) * Math.pow(distance, 4.0)*Math.pow(vy, 7.0) +
        (-0.03994914) * Math.pow(distance, 3.0)*Math.pow(vx, 8.0) +
        (0.00235693) * Math.pow(distance, 3.0)*Math.pow(vx, 7.0)*vy +
        (-0.06107116) * Math.pow(distance, 3.0)*Math.pow(vx, 6.0)*Math.pow(vy, 2.0) +
        (-0.00624000) * Math.pow(distance, 3.0)*Math.pow(vx, 5.0)*Math.pow(vy, 3.0) +
        (0.05368527) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0)*Math.pow(vy, 4.0) +
        (0.00791169) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*Math.pow(vy, 5.0) +
        (-0.07373317) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 6.0) +
        (-0.00682789) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 7.0) +
        (-0.00779036) * Math.pow(distance, 3.0)*Math.pow(vy, 8.0) +
        (0.00885522) * Math.pow(distance, 2.0)*Math.pow(vx, 9.0) +
        (-0.00334612) * Math.pow(distance, 2.0)*Math.pow(vx, 8.0)*vy +
        (0.06310790) * Math.pow(distance, 2.0)*Math.pow(vx, 7.0)*Math.pow(vy, 2.0) +
        (0.00944241) * Math.pow(distance, 2.0)*Math.pow(vx, 6.0)*Math.pow(vy, 3.0) +
        (-0.03438789) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0)*Math.pow(vy, 4.0) +
        (-0.00453712) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*Math.pow(vy, 5.0) +
        (0.02199937) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 6.0) +
        (0.00464270) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 7.0) +
        (0.03035020) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 8.0) +
        (0.00851389) * Math.pow(distance, 2.0)*Math.pow(vy, 9.0) +
        (-0.02206188) * distance*Math.pow(vx, 10.0) +
        (-0.00120013) * distance*Math.pow(vx, 9.0)*vy +
        (0.00143255) * distance*Math.pow(vx, 8.0)*Math.pow(vy, 2.0) +
        (-0.00282529) * distance*Math.pow(vx, 7.0)*Math.pow(vy, 3.0) +
        (-0.00430232) * distance*Math.pow(vx, 6.0)*Math.pow(vy, 4.0) +
        (0.00048984) * distance*Math.pow(vx, 5.0)*Math.pow(vy, 5.0) +
        (0.02309343) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 6.0) +
        (0.00441125) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 7.0) +
        (-0.06129593) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 8.0) +
        (-0.01699187) * distance*vx*Math.pow(vy, 9.0) +
        (0.01493145) * distance*Math.pow(vy, 10.0) +
        (0.00919738) * Math.pow(vx, 11.0) +
        (0.00172075) * Math.pow(vx, 10.0)*vy +
        (-0.00242678) * Math.pow(vx, 9.0)*Math.pow(vy, 2.0) +
        (-0.00042483) * Math.pow(vx, 8.0)*Math.pow(vy, 3.0) +
        (0.00356629) * Math.pow(vx, 7.0)*Math.pow(vy, 4.0) +
        (-0.00113448) * Math.pow(vx, 6.0)*Math.pow(vy, 5.0) +
        (-0.00363207) * Math.pow(vx, 5.0)*Math.pow(vy, 6.0) +
        (-0.00082041) * Math.pow(vx, 4.0)*Math.pow(vy, 7.0) +
        (0.04152151) * Math.pow(vx, 3.0)*Math.pow(vy, 8.0) +
        (0.01243047) * Math.pow(vx, 2.0)*Math.pow(vy, 9.0) +
        (-0.03414660) * vx*Math.pow(vy, 10.0) +
        (-0.00118868) * Math.pow(vy, 11.0) +
        (-0.00004899) * Math.pow(distance, 12.0) +
        (-0.00023135) * Math.pow(distance, 11.0)*vx +
        (-0.00000305) * Math.pow(distance, 11.0)*vy +
        (0.00007452) * Math.pow(distance, 10.0)*Math.pow(vx, 2.0) +
        (-0.00000621) * Math.pow(distance, 10.0)*vx*vy +
        (-0.00002402) * Math.pow(distance, 10.0)*Math.pow(vy, 2.0) +
        (0.00044424) * Math.pow(distance, 9.0)*Math.pow(vx, 3.0) +
        (0.00000988) * Math.pow(distance, 9.0)*Math.pow(vx, 2.0)*vy +
        (-0.00013316) * Math.pow(distance, 9.0)*vx*Math.pow(vy, 2.0) +
        (0.00001920) * Math.pow(distance, 9.0)*Math.pow(vy, 3.0) +
        (-0.00036634) * Math.pow(distance, 8.0)*Math.pow(vx, 4.0) +
        (0.00002870) * Math.pow(distance, 8.0)*Math.pow(vx, 3.0)*vy +
        (-0.00067648) * Math.pow(distance, 8.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.00000067) * Math.pow(distance, 8.0)*vx*Math.pow(vy, 3.0) +
        (0.00033646) * Math.pow(distance, 8.0)*Math.pow(vy, 4.0) +
        (0.00097462) * Math.pow(distance, 7.0)*Math.pow(vx, 5.0) +
        (0.00000538) * Math.pow(distance, 7.0)*Math.pow(vx, 4.0)*vy +
        (0.00051000) * Math.pow(distance, 7.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (-0.00003292) * Math.pow(distance, 7.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.00053739) * Math.pow(distance, 7.0)*vx*Math.pow(vy, 4.0) +
        (-0.00004463) * Math.pow(distance, 7.0)*Math.pow(vy, 5.0) +
        (-0.00005990) * Math.pow(distance, 6.0)*Math.pow(vx, 6.0) +
        (-0.00003563) * Math.pow(distance, 6.0)*Math.pow(vx, 5.0)*vy +
        (-0.00088942) * Math.pow(distance, 6.0)*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (0.00006086) * Math.pow(distance, 6.0)*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (-0.00033626) * Math.pow(distance, 6.0)*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (0.00003954) * Math.pow(distance, 6.0)*vx*Math.pow(vy, 5.0) +
        (-0.00120277) * Math.pow(distance, 6.0)*Math.pow(vy, 6.0) +
        (-0.00064861) * Math.pow(distance, 5.0)*Math.pow(vx, 7.0) +
        (0.00002050) * Math.pow(distance, 5.0)*Math.pow(vx, 6.0)*vy +
        (0.00273182) * Math.pow(distance, 5.0)*Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (0.00001107) * Math.pow(distance, 5.0)*Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (0.00014002) * Math.pow(distance, 5.0)*Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (0.00030369) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (-0.00040879) * Math.pow(distance, 5.0)*vx*Math.pow(vy, 6.0) +
        (0.00009482) * Math.pow(distance, 5.0)*Math.pow(vy, 7.0) +
        (0.00308105) * Math.pow(distance, 4.0)*Math.pow(vx, 8.0) +
        (-0.00015361) * Math.pow(distance, 4.0)*Math.pow(vx, 7.0)*vy +
        (0.00359369) * Math.pow(distance, 4.0)*Math.pow(vx, 6.0)*Math.pow(vy, 2.0) +
        (0.00036889) * Math.pow(distance, 4.0)*Math.pow(vx, 5.0)*Math.pow(vy, 3.0) +
        (-0.00319614) * Math.pow(distance, 4.0)*Math.pow(vx, 4.0)*Math.pow(vy, 4.0) +
        (-0.00052904) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0)*Math.pow(vy, 5.0) +
        (0.00467044) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*Math.pow(vy, 6.0) +
        (0.00035743) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 7.0) +
        (0.00054764) * Math.pow(distance, 4.0)*Math.pow(vy, 8.0) +
        (-0.00084250) * Math.pow(distance, 3.0)*Math.pow(vx, 9.0) +
        (0.00029621) * Math.pow(distance, 3.0)*Math.pow(vx, 8.0)*vy +
        (-0.00571974) * Math.pow(distance, 3.0)*Math.pow(vx, 7.0)*Math.pow(vy, 2.0) +
        (-0.00080593) * Math.pow(distance, 3.0)*Math.pow(vx, 6.0)*Math.pow(vy, 3.0) +
        (0.00312695) * Math.pow(distance, 3.0)*Math.pow(vx, 5.0)*Math.pow(vy, 4.0) +
        (0.00034339) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0)*Math.pow(vy, 5.0) +
        (-0.00247422) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*Math.pow(vy, 6.0) +
        (-0.00030773) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 7.0) +
        (-0.00195468) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 8.0) +
        (-0.00075847) * Math.pow(distance, 3.0)*Math.pow(vy, 9.0) +
        (0.00230519) * Math.pow(distance, 2.0)*Math.pow(vx, 10.0) +
        (0.00011567) * Math.pow(distance, 2.0)*Math.pow(vx, 9.0)*vy +
        (0.00002243) * Math.pow(distance, 2.0)*Math.pow(vx, 8.0)*Math.pow(vy, 2.0) +
        (0.00041778) * Math.pow(distance, 2.0)*Math.pow(vx, 7.0)*Math.pow(vy, 3.0) +
        (-0.00008614) * Math.pow(distance, 2.0)*Math.pow(vx, 6.0)*Math.pow(vy, 4.0) +
        (-0.00003531) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0)*Math.pow(vy, 5.0) +
        (-0.00197065) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*Math.pow(vy, 6.0) +
        (-0.00069524) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 7.0) +
        (0.00587656) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 8.0) +
        (0.00224466) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 9.0) +
        (-0.00162541) * Math.pow(distance, 2.0)*Math.pow(vy, 10.0) +
        (-0.00415847) * distance*Math.pow(vx, 11.0) +
        (-0.00045126) * distance*Math.pow(vx, 10.0)*vy +
        (-0.00048127) * distance*Math.pow(vx, 9.0)*Math.pow(vy, 2.0) +
        (-0.00000084) * distance*Math.pow(vx, 8.0)*Math.pow(vy, 3.0) +
        (-0.00027260) * distance*Math.pow(vx, 7.0)*Math.pow(vy, 4.0) +
        (0.00028474) * distance*Math.pow(vx, 6.0)*Math.pow(vy, 5.0) +
        (0.00070382) * distance*Math.pow(vx, 5.0)*Math.pow(vy, 6.0) +
        (0.00018270) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 7.0) +
        (-0.00804912) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 8.0) +
        (-0.00309159) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 9.0) +
        (0.00698702) * distance*vx*Math.pow(vy, 10.0) +
        (-0.00010668) * distance*Math.pow(vy, 11.0) +
        (-0.00386930) * Math.pow(vx, 12.0) +
        (-0.00043676) * Math.pow(vx, 11.0)*vy +
        (0.00105762) * Math.pow(vx, 10.0)*Math.pow(vy, 2.0) +
        (-0.00039444) * Math.pow(vx, 9.0)*Math.pow(vy, 3.0) +
        (0.00128878) * Math.pow(vx, 8.0)*Math.pow(vy, 4.0) +
        (-0.00010400) * Math.pow(vx, 7.0)*Math.pow(vy, 5.0) +
        (0.00104100) * Math.pow(vx, 6.0)*Math.pow(vy, 6.0) +
        (0.00031954) * Math.pow(vx, 5.0)*Math.pow(vy, 7.0) +
        (0.00014148) * Math.pow(vx, 4.0)*Math.pow(vy, 8.0) +
        (0.00132051) * Math.pow(vx, 3.0)*Math.pow(vy, 9.0) +
        (-0.00528263) * Math.pow(vx, 2.0)*Math.pow(vy, 10.0) +
        (-0.00119214) * vx*Math.pow(vy, 11.0) +
        (-0.00221587) * Math.pow(vy, 12.0)
        ;
}




public static double calculateYaw(double distance, double vx, double vy) {
return
        -13.54643627 +
        (38.77436503) * distance +
        (-3.77582310) * vx +
        (-32.17204504) * vy +
        (-46.15597975) * Math.pow(distance, 2.0) +
        (9.44476381) * distance*vx +
        (-77.04384670) * distance*vy +
        (0.59459408) * Math.pow(vx, 2.0) +
        (-59.91678061) * vx*vy +
        (-0.66992597) * Math.pow(vy, 2.0) +
        (30.01283204) * Math.pow(distance, 3.0) +
        (-9.32303205) * Math.pow(distance, 2.0)*vx +
        (117.06542239) * Math.pow(distance, 2.0)*vy +
        (-1.20967913) * distance*Math.pow(vx, 2.0) +
        (58.16544660) * distance*vx*vy +
        (1.07079549) * distance*Math.pow(vy, 2.0) +
        (0.07619136) * Math.pow(vx, 3.0) +
        (-7.30772299) * Math.pow(vx, 2.0)*vy +
        (-0.21746606) * vx*Math.pow(vy, 2.0) +
        (11.14698014) * Math.pow(vy, 3.0) +
        (-11.71527773) * Math.pow(distance, 4.0) +
        (4.84911108) * Math.pow(distance, 3.0)*vx +
        (-71.79701469) * Math.pow(distance, 3.0)*vy +
        (1.10655320) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0) +
        (-34.99318839) * Math.pow(distance, 2.0)*vx*vy +
        (-0.72350688) * Math.pow(distance, 2.0)*Math.pow(vy, 2.0) +
        (-0.14702908) * distance*Math.pow(vx, 3.0) +
        (-4.24784997) * distance*Math.pow(vx, 2.0)*vy +
        (-0.10137763) * distance*vx*Math.pow(vy, 2.0) +
        (-7.68910055) * distance*Math.pow(vy, 3.0) +
        (0.01334854) * Math.pow(vx, 4.0) +
        (0.84103239) * Math.pow(vx, 3.0)*vy +
        (-0.11560611) * Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (9.45400050) * vx*Math.pow(vy, 3.0) +
        (0.13243606) * Math.pow(vy, 4.0) +
        (2.82146864) * Math.pow(distance, 5.0) +
        (-1.45667932) * Math.pow(distance, 4.0)*vx +
        (23.73777135) * Math.pow(distance, 4.0)*vy +
        (-0.50042472) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0) +
        (13.77244889) * Math.pow(distance, 3.0)*vx*vy +
        (0.27149991) * Math.pow(distance, 3.0)*Math.pow(vy, 2.0) +
        (0.13772175) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0) +
        (5.35569502) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*vy +
        (0.21563748) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 2.0) +
        (1.95986289) * Math.pow(distance, 2.0)*Math.pow(vy, 3.0) +
        (-0.01763290) * distance*Math.pow(vx, 4.0) +
        (-2.46013925) * distance*Math.pow(vx, 3.0)*vy +
        (-0.05037110) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-5.86431523) * distance*vx*Math.pow(vy, 3.0) +
        (-0.14755171) * distance*Math.pow(vy, 4.0) +
        (0.01682557) * Math.pow(vx, 5.0) +
        (0.66878373) * Math.pow(vx, 4.0)*vy +
        (-0.01063043) * Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (2.72283065) * Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.14953376) * vx*Math.pow(vy, 4.0) +
        (-0.77831316) * Math.pow(vy, 5.0) +
        (-0.41065834) * Math.pow(distance, 6.0) +
        (0.25432588) * Math.pow(distance, 5.0)*vx +
        (-4.43128850) * Math.pow(distance, 5.0)*vy +
        (0.11576454) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0) +
        (-3.24403605) * Math.pow(distance, 4.0)*vx*vy +
        (-0.05954652) * Math.pow(distance, 4.0)*Math.pow(vy, 2.0) +
        (-0.05157764) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0) +
        (-1.78917556) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*vy +
        (-0.08215024) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 2.0) +
        (-0.16298613) * Math.pow(distance, 3.0)*Math.pow(vy, 3.0) +
        (0.01302312) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0) +
        (1.11331907) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*vy +
        (0.05845879) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (1.49744467) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 3.0) +
        (0.06072694) * Math.pow(distance, 2.0)*Math.pow(vy, 4.0) +
        (-0.01114710) * distance*Math.pow(vx, 5.0) +
        (-0.48714961) * distance*Math.pow(vx, 4.0)*vy +
        (-0.03141869) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (-1.15018332) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (-0.11401744) * distance*vx*Math.pow(vy, 4.0) +
        (0.45312702) * distance*Math.pow(vy, 5.0) +
        (0.00207932) * Math.pow(vx, 6.0) +
        (0.11580203) * Math.pow(vx, 5.0)*vy +
        (0.00735341) * Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (0.22803267) * Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (0.05146884) * Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (-0.47800635) * vx*Math.pow(vy, 5.0) +
        (-0.00358685) * Math.pow(vy, 6.0) +
        (0.03311193) * Math.pow(distance, 7.0) +
        (-0.02395215) * Math.pow(distance, 6.0)*vx +
        (0.44011732) * Math.pow(distance, 6.0)*vy +
        (-0.01317332) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0) +
        (0.40449032) * Math.pow(distance, 5.0)*vx*vy +
        (0.00704470) * Math.pow(distance, 5.0)*Math.pow(vy, 2.0) +
        (0.00811169) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0) +
        (0.25334338) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*vy +
        (0.01192553) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 2.0) +
        (-0.00931832) * Math.pow(distance, 4.0)*Math.pow(vy, 3.0) +
        (-0.00324360) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0) +
        (-0.19317658) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*vy +
        (-0.01259799) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (-0.18394312) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 3.0) +
        (-0.01032909) * Math.pow(distance, 3.0)*Math.pow(vy, 4.0) +
        (0.00291989) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0) +
        (0.10245403) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*vy +
        (0.01272499) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (0.18562703) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (0.02782754) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 4.0) +
        (-0.09159315) * Math.pow(distance, 2.0)*Math.pow(vy, 5.0) +
        (-0.00092078) * distance*Math.pow(vx, 6.0) +
        (-0.03901973) * distance*Math.pow(vx, 5.0)*vy +
        (-0.00666891) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (-0.02265838) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (-0.02316676) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (0.17034478) * distance*vx*Math.pow(vy, 5.0) +
        (-0.00005445) * distance*Math.pow(vy, 6.0) +
        (-0.00125159) * Math.pow(vx, 7.0) +
        (0.00160253) * Math.pow(vx, 6.0)*vy +
        (0.00161427) * Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (-0.01133990) * Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (0.00791316) * Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (-0.10443309) * Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (-0.00282869) * vx*Math.pow(vy, 6.0) +
        (0.00743246) * Math.pow(vy, 7.0) +
        (-0.00113512) * Math.pow(distance, 8.0) +
        (0.00093971) * Math.pow(distance, 7.0)*vx +
        (-0.01808949) * Math.pow(distance, 7.0)*vy +
        (0.00058604) * Math.pow(distance, 6.0)*Math.pow(vx, 2.0) +
        (-0.02028265) * Math.pow(distance, 6.0)*vx*vy +
        (-0.00034175) * Math.pow(distance, 6.0)*Math.pow(vy, 2.0) +
        (-0.00045255) * Math.pow(distance, 5.0)*Math.pow(vx, 3.0) +
        (-0.01306789) * Math.pow(distance, 5.0)*Math.pow(vx, 2.0)*vy +
        (-0.00059656) * Math.pow(distance, 5.0)*vx*Math.pow(vy, 2.0) +
        (0.00151411) * Math.pow(distance, 5.0)*Math.pow(vy, 3.0) +
        (0.00024333) * Math.pow(distance, 4.0)*Math.pow(vx, 4.0) +
        (0.01205211) * Math.pow(distance, 4.0)*Math.pow(vx, 3.0)*vy +
        (0.00078050) * Math.pow(distance, 4.0)*Math.pow(vx, 2.0)*Math.pow(vy, 2.0) +
        (0.00890323) * Math.pow(distance, 4.0)*vx*Math.pow(vy, 3.0) +
        (0.00061128) * Math.pow(distance, 4.0)*Math.pow(vy, 4.0) +
        (-0.00026295) * Math.pow(distance, 3.0)*Math.pow(vx, 5.0) +
        (-0.00668793) * Math.pow(distance, 3.0)*Math.pow(vx, 4.0)*vy +
        (-0.00117804) * Math.pow(distance, 3.0)*Math.pow(vx, 3.0)*Math.pow(vy, 2.0) +
        (-0.01210490) * Math.pow(distance, 3.0)*Math.pow(vx, 2.0)*Math.pow(vy, 3.0) +
        (-0.00218340) * Math.pow(distance, 3.0)*vx*Math.pow(vy, 4.0) +
        (0.00624445) * Math.pow(distance, 3.0)*Math.pow(vy, 5.0) +
        (0.00016004) * Math.pow(distance, 2.0)*Math.pow(vx, 6.0) +
        (0.00362136) * Math.pow(distance, 2.0)*Math.pow(vx, 5.0)*vy +
        (0.00098892) * Math.pow(distance, 2.0)*Math.pow(vx, 4.0)*Math.pow(vy, 2.0) +
        (-0.00226850) * Math.pow(distance, 2.0)*Math.pow(vx, 3.0)*Math.pow(vy, 3.0) +
        (0.00256715) * Math.pow(distance, 2.0)*Math.pow(vx, 2.0)*Math.pow(vy, 4.0) +
        (-0.01539387) * Math.pow(distance, 2.0)*vx*Math.pow(vy, 5.0) +
        (0.00008826) * Math.pow(distance, 2.0)*Math.pow(vy, 6.0) +
        (0.00023891) * distance*Math.pow(vx, 7.0) +
        (-0.00038589) * distance*Math.pow(vx, 6.0)*vy +
        (-0.00035870) * distance*Math.pow(vx, 5.0)*Math.pow(vy, 2.0) +
        (0.00400648) * distance*Math.pow(vx, 4.0)*Math.pow(vy, 3.0) +
        (-0.00163032) * distance*Math.pow(vx, 3.0)*Math.pow(vy, 4.0) +
        (0.01887143) * distance*Math.pow(vx, 2.0)*Math.pow(vy, 5.0) +
        (0.00061434) * distance*vx*Math.pow(vy, 6.0) +
        (-0.00101763) * distance*Math.pow(vy, 7.0) +
        (-0.00020830) * Math.pow(vx, 8.0) +
        (-0.00208400) * Math.pow(vx, 7.0)*vy +
        (-0.00007121) * Math.pow(vx, 6.0)*Math.pow(vy, 2.0) +
        (-0.00019029) * Math.pow(vx, 5.0)*Math.pow(vy, 3.0) +
        (0.00026747) * Math.pow(vx, 4.0)*Math.pow(vy, 4.0) +
        (-0.00585100) * Math.pow(vx, 3.0)*Math.pow(vy, 5.0) +
        (-0.00039900) * Math.pow(vx, 2.0)*Math.pow(vy, 6.0) +
        (0.00234899) * vx*Math.pow(vy, 7.0) +
        (0.00030662) * Math.pow(vy, 8.0)
        ;
}



public static double calculateAngularVelocity(double linearVelocity) {
 return 2.115421 * linearVelocity * linearVelocity + -24.608210 * linearVelocity + 100.604168;
}

}
