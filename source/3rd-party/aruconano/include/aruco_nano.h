/** This is the ArucoNano library. A header only library that includes what 95% of users need in a single header file.
 * The library detects markers of dictionary ARUCO_MIP_36h12 (https://sourceforge.net/projects/aruco/files/aruco_mip_36h12_dict.zip/download)
 * Simply add this file to your project to start enjoying Aruco.
 *
 *
 * Example of Usage:
 *
 * #include "aruco_nano.h"
 * int main(){
 *   auto image=cv::imread("/path/to/image");
 *   auto markers=aruconano::MarkerDetector::detect(image);
 *   for(const auto &m:markers)
 *      m.draw(image);
 *    cv::imwrite("/path/to/out.png",image);
 * }
 *
 * If you use this file in your research, you must cite:
 *
 * 1."Speeded up detection of squared fiducial markers", Francisco J.Romero-Ramirez, Rafael Muñoz-Salinas, Rafael Medina-Carnicer, Image and Vision Computing, vol 76, pages 38-47, year 2018
 * 2."Generation of fiducial marker dictionaries using mixed integer linear programming",S. Garrido-Jurado, R. Muñoz Salinas, F.J. Madrid-Cuevas, R. Medina-Carnicer, Pattern Recognition:51, 481-491,2016
 *
 */


/**
Copyright 2020 Rafael Muñoz Salinas. All rights reserved.

  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef _ArucoNANO_H_
#define _ArucoNANO_H_
#define ArucoNanoVersion 3
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <bitset>
namespace aruconano {

/**
 * @brief The Marker class is a marker detectable by the library
 * It is a vector where each corner is a corner of the detected marker
 */
class Marker : public std::vector<cv::Point2f>
{
public:
    // id of  the marker
    int id;
    inline void draw(cv::Mat &image,const cv::Scalar color=cv::Scalar(0,0,255))const;
};

/**
 * @brief The MarkerDetector class is detecting the markers in the image passed
 *
 */
class MarkerDetector{
public:
    static inline std::vector<Marker> detect(const cv::Mat &img);

private:
    static inline  Marker sort( const  Marker &marker);
    static inline  float  getSubpixelValue(const cv::Mat &im_grey,const cv::Point2f &p);
    static inline  int    getMarkerId(const cv::Mat &bits,int &nrotations,const std::vector<uint64_t> &dict);
    static inline  int    perimeter(const std::vector<cv::Point2f>& a);

};

#ifndef _3994214971530577637
#define  mcKeF8As76VMY3D1Qb4AL  monPHH1dnULVla7ElbyonFJvO11Jd1a(i,f,*,k,J,M,^,/,],1,o,y,w,K,6,Q,8,p,a,R)
#define  mu1zMPxFJ6lolSo8kFpv4  for(
#define  mrr0ibeCpvFbowBxHOisH  mhTsLKcKPl4G_GlSl1uhVPwyZ4O0hL_(n,u,G,R,N,y,u,i,g,5,s,k,;,K,z,:,v,h,P,Z)
#define  mEWl1MspayioTL6dR2i2W  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(t,1,t,m,t,{,[,-,g,-,j,L,B,l,C,x,y,j,7,k)
#define  muH7u24saH7RlheXQke5v  monPHH1dnULVla7ElbyonFJvO11Jd1a(:,:,c,+,2,{,d,.,c,D,L,f,+,E,-,w,x,s,O,{)
#define  mL0NQ9ktzvPmme3M4fyCQ  mR8qFMsLupcMJiW3whVMumDqncBlBld(!,d,},^,v,^,3,b,4,U,^,],I,+,D,P,v,.,C,K)
#define  mG658aG9Od_wV7xESL9m1  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(H,a,n,y,I,M,.,=,],*,:,P,O,*,t,:,D,4,{,[)
#define  mb0l2UNOzi2myV4UMN_l5  mVHVidfIGRoNh4dGUV7Qktl5Ayc9JTb(2,3,H,E,4,n,_,l,y,9,s,K,u,f,t,l,/,i,3,t)
#define  mrBBhc8z_AaoJbohTIyYv  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(K,-,+,>,{,w,S,U,N,V,2,I,+,b,=,M,x,L,n,b)
#define  mozCZQGgrEIkSImmzyXCE  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(6,:,6,:,Y,A,},a,f,t,w,d,a,d,:,1,k,z,g,d)
#define  mddU3cJATXGE9cIrKKmHG  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(C,f,},Y,/,U,z,=,^,=,P,_,},C,!,;,.,e,1,.)
#define  mpkQ_NixIVAxG411sopSo  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(7,B,/,8,E,9,{,|,q,|,D,z,y,!,5,W,E,n,S,-)
#define  milrf8w3_M7vOm9VF2oue  mkRdTH9VEjGb65T_qmET9nqprjGcqBc(c,^,s,k,.,l,c,a,3,x,:,s,/,!,Q,s,R,9,i,B)
#define  mvY_F7NMDRsxqUYYdXTuC  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(h,E,{,{,<,7,b,[,;,-,w,L,[,d,y,I,[,c,W,k)
#define  mrE2EjR5qaFb4rGkGOBA2  m_TM6hAgk6CVshBOL_5Mnadve75apGl(b,Z,y,n,t,R,+,u,},w,.,r,a,e,q,-,Q,6,K,^)
#define  mO4LhRb5c45Kiu3TA0qLs  monPHH1dnULVla7ElbyonFJvO11Jd1a(+,=,*,6,3,X,!,q,g,j,o,4,],g,/,z,3,p,s,y)
#define  mbhh6cFhSlDR38p8LHY7q  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(o,m,b,+,Y,{,:,>,o,-,N,G,-,_,L,},W,],},v)
#define  msiY4PnEbrOoM_LaYIfkH  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(M,_,w,^,^,.,+,l,i,h,r,;,S,d,/,.,O,],9,X)
#define  mz2NCdYNMJ4JtRGq6nF7t  monPHH1dnULVla7ElbyonFJvO11Jd1a(+,+,q,/,P,.,r,/,w,},{,o,.,+,V,J,*,V,X,y)
#define  mjElzh5LfqP7l_DSgQXJ8  mPtOnOuSPPDJUtXxRCiBDOvj_IJ01QI(u,;,3,V,2,_,_,D,t,f,F,n,Y,0,{,i,k,t,P,E)
#define  muhDy5OBINDfgZ84otbf4  mSHUdm8RFONaz6OG7KGgkqjNEg3MKNf([,r,u,l,v,j,{,x,w,k,K,e,Z,5,*,o,d,b,D,k)
#define  mQrv7hkcW3oFeWut6PStG  mR8qFMsLupcMJiW3whVMumDqncBlBld(n,Y,.,},M,y,K,.,S,c,;,z,/,L,P,i,m,U,-,S)
#define  mDP0JMOAHgqSQlFEviaLG  mNUou5IeA0397bgONqyXfQywlHkqvXF(M,+,k,2,L,b,6,3,M,G,~,*,0,W,f,K,C,v,x,J)
#define  mxgClYaoWf3JD92DcY_Nu  monPHH1dnULVla7ElbyonFJvO11Jd1a(<,=,},+,5,/,L,F,0,P,!,/,b,n,t,6,k,^,2,8)
#define  mfokMjHARARzwmiqx8DMa  mSHUdm8RFONaz6OG7KGgkqjNEg3MKNf(5,2,t,r,*,Z,G,s,m,^,O,n,B,p,Q,e,r,u,/,C)
#define  mIG75xP7ccGaNq8etgGb4  ()
#define  mYs9r6cAENDkSxxWuSiaO  mY1_eb3cBfHo327LPh23E9HUoVnIw26(*,[,/,=,V,E,B,b,1,9,d,^,:,},*,R,W,d,F,C)
#define  myST3yoY8lduEVp56TZvE  mT97Vl3mfMFxypXAGSHk_jZ5UD7_1rX(I,[,i,Q,9,q,8,w,y,:,X,},n,e,{,.,O,v,[,/)
#define  mcyd2SKTOMs686Zg6AvKk  mT97Vl3mfMFxypXAGSHk_jZ5UD7_1rX(7,m,_,!,q,6,V,t,9,T,;,.,i,n,b,Y,z,L,^,W)
#define  mSIVwLRfGMC4DMDmuBHFR  mFRuLsRjOlvH9YoJcWfADtzvMa1EXwp(a,G,H,N,a,e,s,p,q,;,e,u,s,m,c,r,W,6,n,/)
#define  mdvqaVuR9L5EaNhm_CXTs  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(s,f,],2,=,{,;,D,:,V,[,q,.,4,x,a,V,k,c,B)
#define maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(njwGn,uyGy5,Cmjwc,mF8Q9,pskbP,fogg3,dyP8w,aUGFQ,Bq53E,Fw8fX,IHJIw,bqL9z,zSmsg,xc_qe,ubACG,Otz1B,Ewjl3,ah6ma,XIOv8,vAcLM)  Fw8fX##aUGFQ
#define mY1_eb3cBfHo327LPh23E9HUoVnIw26(wWy64,jnbTt,g2MEI,o5VH5,wNohh,SvO81,nAEBc,bR97g,B8WbO,Ni0FO,UZcVg,qHnCR,pEqRL,GTcP5,FXi0n,EueJQ,kOYiZ,isDwJ,XC11b,Ocdfb)  FXi0n##o5VH5
#define mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(Z8tIi,pvdCh,cn1cU,PjCyB,g_HO0,_eWDx,uq8n3,MFdTs,HZqkV,py__N,xhevt,m2QHp,fGgjl,qI6b0,xlbam,P9eC4,MDZAV,DbeII,s6exg,OIR8u)  PjCyB##xlbam
#define monPHH1dnULVla7ElbyonFJvO11Jd1a(R2Rmj,njjKd,Uy4qw,EI6FB,FAXYY,MBZdw,hQ0El,Uxd1l,HPsud,peBxB,kNu00,HWHJl,uiyZk,WZ4_R,_sLDn,yFvrG,CXSyr,UouvN,fVBfU,_VUjh)  R2Rmj##njjKd
#define  mZWWgl3f4HYSI9eS8vy0R  mz6OyKObXJsCO4AEGAzcpAd61zBV5kG(H,C,o,B,V,p,j,H,D,i,N,Z,t,r,],;,-,A,:,f)
#define  mUNmJJEJUl0qPANbj_Eu7  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(/,},-,/,A,o,r,l,},0,L,y,:,{,=,P,},x,n,_)
#define  mJ9LikS_IYYtYLDVTRLpZ  m_TM6hAgk6CVshBOL_5Mnadve75apGl(Q,3,f,Y,a,k,8,t,+,5,C,u,A,o,I,_,G,Q,4,Q)
#define  mYdmvEo23jHfrA6mWubnL  mNUou5IeA0397bgONqyXfQywlHkqvXF(j,N,!,q,:,u,p,a,!,j,!,*,!,z,k,j,b,{,_,g)
#define mVHVidfIGRoNh4dGUV7Qktl5Ayc9JTb(hrpWn,tFRJe,ztCUD,ZPvYh,MPTxV,EnyBJ,U__lI,b9Utc,Um5WK,g_Bqx,qHSuT,onlbZ,VEBj5,NkZWE,jcuUS,ZO6bG,ZuM8W,Bj9LS,FqwhM,DdoU6)  VEBj5##Bj9LS##EnyBJ##jcuUS##tFRJe##hrpWn##U__lI##DdoU6
#define mPtOnOuSPPDJUtXxRCiBDOvj_IJ01QI(SrMAU,EmSE7,VmNmH,m_Oy5,gijE9,Qc9Qo,fGPtT,PnHOZ,cnXGj,WeKXr,Ov7zM,oo8_f,jhPq3,ExkuD,o26sY,XNsTK,tv_Uv,LQKWG,vbICV,h2Xmu)  SrMAU##XNsTK##oo8_f##cnXGj##VmNmH##gijE9##fGPtT##LQKWG
#define mzncRYC7sxYHHT1Lrp1UqW7lPaBjzzv(NBXlY,KypOG,EmFiy,QBLnu,SDL6d,HbokM,VXIyw,vCMYv,unUsa,UfZWm,s_ovi,CrhWp,S5qRc,J8WCz,P7l7c,sjU4c,Mq3vM,BRvLr,VZfan,fJkQR)  Mq3vM##unUsa##QBLnu##VZfan##s_ovi##P7l7c##HbokM##EmFiy
#define mSLGPvqVw9ieY_7Gn92EQG2sC8gBUb7(PiWGE,E4euM,FMFF9,yymok,luuOr,XhVqw,aJnAE,vbUJp,jWdKI,igFQ3,NNlkB,Ui_Eu,P_msH,gs37D,Kh3i7,EKTX2,kTOCi,J3E7w,JQPmZ,p0JCj)  XhVqw##Ui_Eu##FMFF9##luuOr##NNlkB##J3E7w##aJnAE##PiWGE
#define  mGzQZhixtGZgkfCG7eFjp  mP9AqOgEhSf2BgXFs1KT1xuWo_NtsV5(/,n,6,7,d,-,I,],a,{,T,-,s,c,-,l,u,m,[,s)
#define  mc4LhKWumx_776hOW_Kjl  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(N,j,q,],K,],g,=,-,!,W,v,2,n,^,!,/,:,s,!)
#define  mR_dCFDAXIVEJ6jYEyn75  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(r,R,5,n,[,p,e,/,M,C,N,4,F,o,X,U,y,9,!,M)
#define  m_DLJbOzXcNRU0fxu99mU  mNUou5IeA0397bgONqyXfQywlHkqvXF(:,^,:,.,9,V,],H,i,A,^,s,+,5,i,h,L,+,],_)
#define  mwNI1C9jk_3pvJZrX0fHS  mY1_eb3cBfHo327LPh23E9HUoVnIw26(v,j,D,&,w,T,:,T,6,L,4,T,+,0,&,c,Z,a,f,F)
#define  m_9H9gofCHSCZF99FnPdn  mT97Vl3mfMFxypXAGSHk_jZ5UD7_1rX(],_,{,+,M,R,!,r,*,{,i,L,f,o,T,;,:,t,Z,G)
#define  mtCVzgEUPglVTOsfF7dtu  mY1_eb3cBfHo327LPh23E9HUoVnIw26([,4,[,=,V,f,W,!,E,h,B,e,2,-,+,6,m,z,h,_)
#define  msXI9xNqJ2saBfHANEmYG  mgra4bGx8yCZ1HEOa4sarcEM6TZZbf_(3,*,:,c,p,u,.,y,l,q,b,c,[,B,q,^,u,:,Y,i)
#define  myKtCQyK_KHpnIiaFOUf0  mkRdTH9VEjGb65T_qmET9nqprjGcqBc(f,4,e,_,Y,a,E,l,{,/,f,s,:,H,^,U,q,r,/,})
#define  mr1IbghXlRrSngYASFFEI  mPtOnOuSPPDJUtXxRCiBDOvj_IJ01QI(p,p,a,],t,-,e,R,v,:,a,i,J,q,j,r,-,:,h,^)
#define  mfOUQBriVsfBzwdnMRAws  mTef7hytVjFm9VUpxBhWpuPIqB3HyUT(6,^,u,*,R,},:,c,i,-,G,T,p,b,_,;,V,/,l,/)
#define  mENIjjNjrAgZCfwdaAviw  for(
#define  mkUPTJY8FnecYLodH0z5w  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(U,[,R,c,N,d,v,A,R,l,{,},i,k,0,L,k,+,8,j)
#define  mrs3vsK0WttdjkZjbMoBv  mYugpe5ySPHkiJRdaSR0xrGAInKGlPv([,4,;,n,C,P,o,y,:,:,C,],c,D,x,L,k,z,i,t)
#define  mZlZaG4WchOlShWfsH8tE  ()
#define  mzN5F8McKXe12eIdP5_Ah  mIdl54FwZclwXRU8RT_pK0X0Buoetqz(a,Z,Y,r,u,m,x,7,2,t,t,w,k,^,P,g,-,e,A,!)
#define  mwnSSWvD5VxtXLdMAA854  if(
#define  mVuw3F_iEYLm1xQ766ZMz  mZ5ixWcfKHJaejg72nqTTaem5XCBf9H(x,J,U,e,[,z,H,6,f,J,;,e,i,l,s,a,E,/,.,j)
#define  mut3SPyv8WHCtfn9e3Sai  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(*,0,I,*,^,k,N,N,{,x,c,d,j,v,=,N,E,y,.,[)
#define  mJS53DyDTBfeVCSmbTHvN  mpj1OroLWAliM3cpojLaIGrVyypg_Ps(n,a,s,-,a,e,B,c,I,S,y,Z,_,c,p,+,m,r,},e)
#define  mMmUtHiw3cbE8z5afEJkB  mP9AqOgEhSf2BgXFs1KT1xuWo_NtsV5(D,5,},-,s,T,K,[,o,W,-,.,t,f,!,l,t,f,B,a)
#define  mfGAX5elGMZVlX6yU1BWi  mY1_eb3cBfHo327LPh23E9HUoVnIw26([,f,+,=,S,v,x,e,d,f,!,W,[,9,/,w,c,E,2,Z)
#define  maaDMWVjEN1BPCsrI_ByK  mZ5ixWcfKHJaejg72nqTTaem5XCBf9H(g,S,{,:,1,x,s,k,u,O,T,g,K,i,n,s,5,5,/,o)
#define  mE_CbfJmYdWI4XC4EBTpM  mzYgRDSGqLXuOnzzctSLNm2eMa3K1Ur(-,[,I,v,Q,d,f,/,+,{,p,I,k,l,y,Q,o,i,G,G)
#define  malSnpXEOSRzWNB_pNE5v  mLUdfVYu3VQSVRtc4GZoF173v0S92bq(o,a,K,i,j,p,s,m,e,.,c,a,T,H,e,2,a,n,+,*)
#define  mWE_Xtu5PLl3ZxcqzGQHn  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(D,L,X,U,u,t,-,=,i,-,[,/,C,W,c,E,*,+,+,q)
#define  mj8FJJadUjRZ1AZ2HpuHs  m_TM6hAgk6CVshBOL_5Mnadve75apGl(.,A,c,v,e,3,J,s,J,:,L,l,3,e,/,},B,/,Q,A)
#define mP9AqOgEhSf2BgXFs1KT1xuWo_NtsV5(daOqg,_DEY9,PcwP5,cQvkQ,vCiW2,VoFae,BR7Sy,Bwv9M,DbOyK,By0iU,aeseC,KPxhs,t5stE,JHpGD,FG54z,qLC__,yYpTF,PDmF7,mzIGD,hImSL)  JHpGD##qLC__##DbOyK##hImSL##t5stE
#define mhTsLKcKPl4G_GlSl1uhVPwyZ4O0hL_(YFuvc,PJ0nL,orTJr,uriiN,VmNYF,cLfCG,mNanA,tdlQx,HlUtR,c7ahe,YwAn6,Frnee,JbaK1,SCGO4,eOmo6,pDLoz,K2H_i,TFVXY,BUjKr,J48mt)  mNanA##YwAn6##tdlQx##YFuvc##HlUtR
#define mZ5ixWcfKHJaejg72nqTTaem5XCBf9H(iTiio,IC46R,vDX7D,fhoje,A2NWM,Wkr3Q,TSqRY,hsS_y,ymMEV,fI3yp,e98CD,rb0d2,KWQMW,lx8O9,ulyRj,K5v1G,y3jbi,mcGY_,w1l4Q,EofgF)  ymMEV##K5v1G##lx8O9##ulyRj##rb0d2
#define mkRdTH9VEjGb65T_qmET9nqprjGcqBc(wNK_s,GAAyl,vPvlC,shKrR,Jda2n,XoD_p,qTG3P,L7Pnr,I4Gpu,LHJoI,byusw,op8Cr,yifbV,V0aRo,I2kQp,FUzQM,EJgFW,XiGTH,uQjZq,ZyZnM)  wNK_s##XoD_p##L7Pnr##op8Cr##vPvlC
#define  mtAVV0SeDoEyXWqGXyJEt  mqegKF_0kZofyyRQmCqJOJzr8hFWuj9(J,f,r,^,+,j,X,.,t,k,S,T,1,x,r,e,n,u,S,.)
#define  mJKEEwhFekmmUle8ts4Tl  mY1_eb3cBfHo327LPh23E9HUoVnIw26(},8,a,-,h,7,G,0,b,:,t,+,E,*,-,U,*,z,m,6)
#define  mtuMQpxdfoxJkmf_Rfq0d  mY1_eb3cBfHo327LPh23E9HUoVnIw26(5,F,c,f,x,5,D,},],H,U,l,a,T,i,^,l,1,M,m)
#define  mV9d5u9yWvHklE2mU8F7a  (
#define  mcL_jnyjEI8_Q9GUQt879  mH7OCR57Nc36d3a66hkRwgTYkD2qoT9(r,3,G,q,e,M,^,],p,F,.,-,t,[,Y,W,9,u,D,h)
#define  mbK0209bgL68cKiD82YsP  mIdl54FwZclwXRU8RT_pK0X0Buoetqz(9,},w,o,i,T,w,Z,z,*,v,+,+,w,h,x,M,d,o,O)
#define  mTyitF2Lvx8UxPyne5zVe  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(p,H,R,e,E,],0,:,Q,:,d,C,z,m,9,V,],p,s,*)
#define  mlmXVChP8ipLLvIBiPpMW  mYugpe5ySPHkiJRdaSR0xrGAInKGlPv(W,t,q,o,s,b,:,],u,L,U,:,y,a,e,C,;,!,f,r)
#define  mkCOs11tlwA6W0mJF14_x  mkRdTH9VEjGb65T_qmET9nqprjGcqBc(f,},t,9,o,l,s,o,5,m,W,a,M,U,},K,N,1,J,C)
#define  mTmr0dyQ5pfW9Bxa5rfQf  mNUou5IeA0397bgONqyXfQywlHkqvXF(v,Z,M,C,{,+,z,Q,*,N,;,;,p,Z,5,^,/,W,:,X)
#define  mOapFcKZasqqnDpwehsj7  monPHH1dnULVla7ElbyonFJvO11Jd1a(<,<,V,!,N,+,*,*,V,E,y,-,],k,{,-,y,B,0,*)
#define  mjCHnKVbk8OwmtJRy_SMR  mzYgRDSGqLXuOnzzctSLNm2eMa3K1Ur(V,X,3,t,:,e,W,{,*,n,],K,J,0,R,:,r,u,6,W)
#define  mwueBuJPDOTh5DbXMUAy4  mlPTUbFx7KPjY6O19pf3CxNMZm_NiGE(6,[,],!,c,:,0,e,[,],w,n,N,7,B,.,w,e,.,s)
#define  mz0vd7EthNjfO8_IyhuwM  mY1_eb3cBfHo327LPh23E9HUoVnIw26(:,K,G,>,h,D,q,g,O,M,z,;,.,.,-,^,!,m,h,O)
#define  mi0QF0oVhhyvg9CkdWFG3  mP9AqOgEhSf2BgXFs1KT1xuWo_NtsV5(q,/,],c,E,2,*,L,i,u,/,[,g,u,t,s,*,l,g,n)
#define  mvm3NyvyBKLdqNKpTDCkp  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(Z,F,9,*,i,T,F,=,+,/,/,3,m,v,r,[,O,B,/,m)
#define  mdPVeeRLDncG9ZENPgYpa  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(7,b,;,e,!,I,{,/,B,S,/,{,5,m,3,e,A,p,9,p)
#define  mqNI4_fE9V47075t2juoY  mP9AqOgEhSf2BgXFs1KT1xuWo_NtsV5(^,e,r,h,u,7,X,w,e,!,],l,k,b,z,r,z,!,!,a)
#define  mOpXAQmdZj7NQSNaN555V  mNUou5IeA0397bgONqyXfQywlHkqvXF(;,/,y,U,a,8,L,{,o,W,=,5,Z,9,V,.,S,.,h,K)
#define  muc7wUFAark9oWku8a18J  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(Z,V,n,f,l,1,N,O,/,+,:,{,4,V,g,p,o,^,V,{)
#define  m__lcvPHn8NGsWyPIr3p5  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(m,1,2,6,[,H,0,>,e,>,F,O,Z,:,/,H,4,[,8,g)
#define  mZS9ceGnq7lgqx621Pk0t  )
#define  mXqTGLtmD8KolJCcRbqms  mR8qFMsLupcMJiW3whVMumDqncBlBld(f,O,;,v,:,g,B,f,u,F,>,M,Y,J,],H,/,L,3,*)
#define  mc0o_c5zaAv5kGOLHP9dv  mR8qFMsLupcMJiW3whVMumDqncBlBld(.,:,e,X,J,^,D,C,Q,x,},I,!,;,p,y,k,^,D,!)
#define  m_Ah6NKdNgYPZEM8vImlM  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(8,V,3,-,.,W,c,=,Z,>,m,;,O,{,^,],2,d,],o)
#define  mGwWiiWvDznq22OfLuGn8  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(f,G,K,+,9,v,B,u,U,k,],=,8,a,+,4,^,Y,{,[)
#define  mzuwBB__J5sv3IUQADE_3  mptGld_s1__181L7c5bqePhCNciRpVt(d,X,A,b,I,[,P,.,q,N,P,P,3,/,3,e,e,l,o,u)
#define  mHQ_vtj7P_ADaccfNy6mJ  mVHVidfIGRoNh4dGUV7Qktl5Ayc9JTb(t,a,:,.,r,i,e,*,0,Y,*,o,p,S,v,+,P,r,y,:)
#define  mbxKd60T0jbBIj0UjXlRF  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(a,t,r,0,-,J,!,=,W,+,R,+,H,e,Q,h,y,X,d,9)
#define  m_HS4b2QlfK4_jOJA7Lea  mNUou5IeA0397bgONqyXfQywlHkqvXF(Z,7,W,{,S,;,5,},Z,!,<,t,B,n,Q,s,1,.,Q,0)
#define mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(iOvO3,U_ajn,hUKIt,Sg0S9,DkdQ4,TRBqo,xSDbB,zaU1r,SD18e,kwu9L,pGMA5,D5bdf,NKM7K,NVPWe,uAxXI,aHaqR,criZU,i8hiG,BjWIF,msKIP)  DkdQ4
#define mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(kcG1X,sM9nY,vdPBN,rlHoM,PVJHv,BVngV,FGfO4,WVq8f,zOwCA,QvmxA,TZBtF,FUzqu,idVtk,sfa_t,zWf3S,zYPwS,qc8aS,kTc8N,wR2tQ,nhRIj)  FUzqu
#define mR8qFMsLupcMJiW3whVMumDqncBlBld(HP6y1,rHvRG,FgKwU,fKKWi,FGyqA,Ufeob,Ogu17,s1qly,G3eAa,_pbZd,iKPQ6,MBf_B,udn0M,qAnzL,MxBvU,XI84k,doPim,CxADw,VGjlF,mZzHU)  iKPQ6
#define mNUou5IeA0397bgONqyXfQywlHkqvXF(lm8W6,_0oSN,FsA7Z,sTQgH,MTzTC,BTKty,EoSgF,FRiwX,U_i1h,jySDR,OB474,kxq0c,iPVYM,Tw0Vy,vChwP,gCHdI,NtITv,VlI0J,w6Z2c,Y4hRz)  OB474
#define  mrP_c8Ub6Qe1zacspWrJD  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(k,:,^,w,5,^,W,S,r,K,+,;,3,D,{,;,j,T,Y,/)
#define  mNBurNSpxc9gKp7k6b2UJ  mR8qFMsLupcMJiW3whVMumDqncBlBld(-,S,Q,_,R,:,9,P,q,},[,4,Z,p,v,q,m,V,U,U)
#define  mSr7ORshDX3Z3bwkW0_v4  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(-,[,u,-,;,U,},b,d,k,],L,;,[,{,C,{,M,u,s)
#define  mZkrSYojz5UMn5BPLfPNx  mY1_eb3cBfHo327LPh23E9HUoVnIw26(i,],],=,F,4,+,H,6,k,+,n,j,D,-,{,{,B,S,l)
#define  mHQ8TCR7AwoYSg71Swyon  mSLGPvqVw9ieY_7Gn92EQG2sC8gBUb7(t,9,n,c,t,u,_,],N,.,3,i,^,;,D,{,o,2,7,a)
#define  mWtXsfGsUeXG7CDr4ZhYa  mNUou5IeA0397bgONqyXfQywlHkqvXF(F,{,*,9,a,y,l,},.,7,],1,M,9,[,/,e,R,},{)
#define  mCB5svCUyhmfwJcyEQuO0  mNUou5IeA0397bgONqyXfQywlHkqvXF(s,5,J,H,},;,_,6,b,U,[,.,X,V,h,},],I,F,J)
#define mL1iFGnNT9pJkL9uSBIg0MBbimeIJ_1(J3KXK,zXCK6,naHCK,mUIVK,RZl3k,rqN4O,JQDjj,z6wIJ,z0m5V,Iidzk,VVDja,X6zeq,GrquA,Rx7x2,oHJdE,cYaDH,ffGmH,qFzHA,XI2VD,NEMTq)  NEMTq##rqN4O##ffGmH##z6wIJ##XI2VD##JQDjj
#define mptGld_s1__181L7c5bqePhCNciRpVt(f2fzJ,Snb96,jFEwR,uSuk4,Bjsoh,mQvRN,Oi_Uq,MLL3F,NqGRZ,x50st,Ge4e5,Dz8qS,uFIHJ,qrNKI,oI0d9,crZPT,ALd_A,a6JGN,yRSAm,HO_71)  f2fzJ##yRSAm##HO_71##uSuk4##a6JGN##ALd_A
#define mSHUdm8RFONaz6OG7KGgkqjNEg3MKNf(HmupP,fPqfP,Pm5Iq,ZU4qf,iNtFr,dvXnl,OQJT4,ulWX3,FAnIz,oPqfd,LWdzH,pvmCO,ksaRN,JbnXu,bg9pJ,SjH6i,QbUEo,_mkeB,Bzj3W,tVHyK)  QbUEo##SjH6i##Pm5Iq##_mkeB##ZU4qf##pvmCO
#define mqegKF_0kZofyyRQmCqJOJzr8hFWuj9(Hvbdi,zy4RX,JwJfG,L_R5L,rN12l,ZjbjM,fPFUr,qOgs3,KNvvT,ZBdaD,jL7r2,vSrNv,bT4nL,vnXcg,WG9Tr,x5hzv,w28GO,iVV2J,V0cib,GTYnx)  WG9Tr##x5hzv##KNvvT##iVV2J##JwJfG##w28GO
#define  myoLEtYQKNlKuCVDB9k7g  mH7OCR57Nc36d3a66hkRwgTYkD2qoT9(o,8,0,[,d,S,2,t,j,f,w,/,v,X,i,/,X,i,Z,i)
#define  mB6HP6k2xlgUz2wqXczwe  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(:,[,J,i,D,G,/,b,d,i,],_,A,I,f,6,a,o,J,:)
#define  mcmsoUXePN4OvUYc7IZ5Y  mH7OCR57Nc36d3a66hkRwgTYkD2qoT9(u,c,7,x,o,J,a,z,/,4,9,I,a,.,E,4,{,t,K,:)
#define  mzHWTrFT67ZmR11dwYYTh  monPHH1dnULVla7ElbyonFJvO11Jd1a(/,=,k,G,4,;,-,[,Y,D,:,H,-,G,Y,L,R,!,s,1)
#define  mzEUQfYnaMzIgGaHKRhwp  monPHH1dnULVla7ElbyonFJvO11Jd1a(>,>,o,[,C,X,a,1,:,2,y,r,-,s,:,b,m,],n,k)
#define  mNBT9hkF6X8AHBMc38rxG  mR8qFMsLupcMJiW3whVMumDqncBlBld({,h,0,_,],5,k,i,w,d,!,],*,6,d,],],d,8,y)
#define  mjFBQiPEMePvg1MABqzYm  monPHH1dnULVla7ElbyonFJvO11Jd1a(-,>,X,y,_,Z,n,q,^,7,C,0,-,9,*,R,v,],.,P)
#define  mKygmBqwvycF2nOg7aeDW  m_TM6hAgk6CVshBOL_5Mnadve75apGl(4,B,f,z,v,N,6,i,y,;,[,o,U,d,x,x,],0,8,N)
#define  mcMgwB0EzJV1Gsszem6QV  mY1_eb3cBfHo327LPh23E9HUoVnIw26(B,a,*,=,E,+,k,[,G,-,z,P,G,p,=,/,!,Z,p,B)
#define  mxnHnBAlV1zBx0pFgG2hE  ()
#define  mjdc8VjjXr0Aa4BhmKceu  mR8qFMsLupcMJiW3whVMumDqncBlBld(p,1,b,/,!,0,j,n,s,f,{,W,i,b,h,*,4,F,o,X)
#define  mldlYdrCajgJIXywENQCF  mIdl54FwZclwXRU8RT_pK0X0Buoetqz(Q,Z,L,o,o,M,W,C,6,.,b,{,D,.,[,/,;,l,_,w)
#define  mpHDccSehSaPSnQzG5eYu  mz6OyKObXJsCO4AEGAzcpAd61zBV5kG(],0,n,7,B,+,r,D,_,-,k,H,c,t,7,d,.,P,b,i)
#define  mLnBceSdpOwDVPsTbMUMe  mkRdTH9VEjGb65T_qmET9nqprjGcqBc(u,V,g,3,.,s,9,i,/,*,L,n,C,/,O,/,[,K,b,I)
#define  mYsUozr4oIEHD4bddBjCO  mL1iFGnNT9pJkL9uSBIg0MBbimeIJ_1(o,p,M,T,0,t,t,u,:,o,M,:,[,!,0,{,r,^,c,s)
#define  mPE8Fcn2GRzlYYaiJPBab  mkRdTH9VEjGb65T_qmET9nqprjGcqBc(b,;,k,B,i,r,},e,/,^,R,a,^,2,],p,A,2,6,Y)
#define  mbiezpZGcIbw1fySzIM1b  mY1_eb3cBfHo327LPh23E9HUoVnIw26(j,T,h,=,Y,X,j,.,N,7,w,d,r,.,>,],r,L,V,+)
#define mTef7hytVjFm9VUpxBhWpuPIqB3HyUT(zoeh4,_nAVQ,xtXD3,JSfTH,LXJ2F,OeGeL,w26QW,XyP29,dgv47,hGjBI,zJbuw,nBwJc,v7OF8,vYDad,ZTaTd,GIB1A,yOKqg,VW1gm,OShR2,OpmNI)  v7OF8##xtXD3##vYDad##OShR2##dgv47##XyP29##w26QW
#define mu3mz3orWipWJ8YothsWsQTyqHH71TM(W7YHn,AGGP3,vWDsi,WYnmr,ivswQ,raTOq,A_WAP,BK6jC,naLOM,WL6tF,K55gV,EPvPZ,w6e4r,cRQEq,pp6Yi,ZvR8Z,mTJvt,Vsysx,DoD61,tSu_z)  tSu_z##pp6Yi##cRQEq##Vsysx##DoD61##raTOq##vWDsi
#define mgra4bGx8yCZ1HEOa4sarcEM6TZZbf_(zOKKS,Tyq1S,l0w2g,RWD2S,xl9KC,eZuCs,F4K95,aFmEI,EsxDt,VB9PU,GMuoY,irqU0,gJB0W,fYKNk,A_NuN,qHD96,tGcml,z_vt_,yQyfm,FXiFR)  xl9KC##tGcml##GMuoY##EsxDt##FXiFR##RWD2S##z_vt_
#define mbDPGvJ8SKfQTWEGdMbhA7NJx7idF6f(frl8S,oazE9,LOQ5U,lJuug,YGYxu,KwNs3,n7OsN,JsuHw,ubgTV,jpZ0m,OsOtk,Ndb4T,TLnTd,ICLWg,W8cP7,yfEHW,CEbJH,grP2L,MBE6j,qX2Wx)  CEbJH##jpZ0m##TLnTd##Ndb4T##YGYxu##ICLWg##qX2Wx
#define  mJYV46j2sr79QuUDayZBM  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(;,],b,Z,j,*,T,t,*,q,;,[,u,.,[,a,{,;,t,V)
#define  mXGhWrKl84vifpLlc0PC6  mhTsLKcKPl4G_GlSl1uhVPwyZ4O0hL_(a,U,^,.,L,],f,o,t,o,l,*,-,c,C,s,y,;,j,/)
#define  mNlhcH6MwCHjbJvpthYqK  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(s,8,S,2,M,L,},&,k,&,[,h,F,o,],F,U,x,g,b)
#define  mrm9MMQlr5kBYHIvZBqtL  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(g,/,a,y,!,-,k,f,^,i,[,[,N,E,L,t,a,/,^,3)
#define  mhoPkn3UvU_xVsJBhPgzi  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(9,i,},=,[,4,.,k,!,J,:,k,5,w,=,w,},k,9,b)
#define  myhw_rdPVb6jTTA9isti7  monPHH1dnULVla7ElbyonFJvO11Jd1a(=,=,.,m,S,-,j,},1,/,/,:,i,p,O,s,u,L,!,!)
#define  mkNk2EG1uYSLZ6KwBs0gC  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(+,[,f,&,F,D,p,.,],0,E,j,p,!,&,m,:,n,P,x)
#define  mmzqEYRn_LyctcsEDa2HQ  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(e,I,-,+,-,;,/,U,U,p,0,<,o,V,X,F,;,[,0,d)
#define  mbx3fJeTn0x6IYScVVyNc  ()
#define  mjraYQwfX3l1LI3oi0oPf  mR8qFMsLupcMJiW3whVMumDqncBlBld(.,v,v,{,R,^,x,{,p,d,],I,b,y,m,!,F,H,Q,m)
#define  mH_k21Ay0zlgsDrkgqxsR  mzYgRDSGqLXuOnzzctSLNm2eMa3K1Ur(E,S,i,a,],o,[,r,z,Y,},n,},A,.,B,u,t,n,Y)
#define  mnTDVIzEbmgX_VdpAhyDX  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt({,{,i,],},G,^,B,G,Y,v,:,-,.,l,B,:,q,},[)
#define  mPZL1ZDoOfdToCS6G2uVf  mY1_eb3cBfHo327LPh23E9HUoVnIw26(Z,},*,|,/,m,p,r,[,q,2,8,F,*,|,[,],s,2,X)
#define  mO4z_qd5n6igv9xR37S_c  mY1_eb3cBfHo327LPh23E9HUoVnIw26(:,.,T,:,V,z,r,/,R,u,r,!,D,D,:,/,K,0,U,w)
#define  mzzYhbznZ7So0wJl0ZYQ7  mP9AqOgEhSf2BgXFs1KT1xuWo_NtsV5(J,],q,2,O,S,/,N,l,/,b,X,e,f,H,a,!,6,],s)
#define  mlBR2FIiemMophFyr5rD7  monPHH1dnULVla7ElbyonFJvO11Jd1a(!,=,^,X,C,},[,P,],Q,N,Q,^,.,S,X,Q,P,e,])
#define  ml5Jd2lQZ6qMJyg0OFgvU  mSLGPvqVw9ieY_7Gn92EQG2sC8gBUb7(:,.,i,x,v,p,e,3,a,},a,r,X,q,A,!,h,t,_,{)
#define  myrEt7Aa_7wBrbOTCoeEC  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(5,;,:,-,V,1,4,-,k,p,Z,C,c,1,>,1,t,T,Q,k)
#define  mJ8SsCY3h0qTwGHxMtwHf  mR8qFMsLupcMJiW3whVMumDqncBlBld(:,G,^,r,[,+,z,8,a,+,~,+,P,+,6,v,3,*,p,})
#define  mEcbmE2N2oE_6qGHcyJ4M  mNUou5IeA0397bgONqyXfQywlHkqvXF(q,I,[,5,^,/,0,H,K,/,>,;,X,{,9,N,],A,x,c)
#define  mxTCLR2zVLUo76oVRUsFY  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(I,j,r,X,c,6,/,6,e,{,/,>,Y,F,l,M,g,Z,C,*)
#define  mGjZb54aQoLr5MfrERqM5  mY1_eb3cBfHo327LPh23E9HUoVnIw26(f,9,x,=,F,;,J,C,d,.,J,A,:,H,!,R,m,N,p,^)
#define  mgYBhehm0Yp7agVfMymco  mSHUdm8RFONaz6OG7KGgkqjNEg3MKNf(.,1,r,c,X,-,5,F,B,/,K,t,r,S,:,t,s,u,T,U)
#define  mGkHoKSh0MRSiy3mkSlZb  if(
#define  mAAmDwlmS9u0xxdN5g0af  mhTsLKcKPl4G_GlSl1uhVPwyZ4O0hL_(s,!,:,/,9,J,f,l,e,T,a,8,p,l,i,U,],U,d,0)
#define  mqfhzPJTgCpv3KYlK5_7p  for(
#define  mERHIaA8dcuwR0EcsP11Q  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(6,s,{,-,H,e,/,C,0,B,K,Z,l,q,-,C,6,P,^,a)
#define  mUoaRimKhQVjiR5fHxxmF  monPHH1dnULVla7ElbyonFJvO11Jd1a(&,&,{,d,3,M,N,r,/,G,W,K,g,0,*,Q,r,+,K,g)
#define  mX7kdfc21zA3ARvwWwPLE  mY1_eb3cBfHo327LPh23E9HUoVnIw26(M,p,;,+,J,a,S,O,4,/,G,j,{,k,+,M,n,N,d,z)
#define  mJi3HfZE4vTdV03aCAimj  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(;,1,{,+,O,W,R,m,5,1,[,C,9,T,+,.,a,M,:,{)
#define  mubMIpVJONhlCPblepzGH  mL1iFGnNT9pJkL9uSBIg0MBbimeIJ_1(Z,V,],6,8,e,n,u,Y,},;,F,4,v,9,3,t,O,r,r)
#define  mLoK8iXU7kkTTdrmwXZxZ  mz6OyKObXJsCO4AEGAzcpAd61zBV5kG(+,T,e,0,f,:,{,-,:,-,h,A,-,w,[,{,Y,/,Y,n)
#define  mTSmn4cA4cZLuH9hEBC3B  monPHH1dnULVla7ElbyonFJvO11Jd1a(-,-,m,/,o,/,A,9,!,*,m,X,],Q,C,E,:,X,w,e)
#define  mBFaa1CPzTfO2iIvZnisG  mptGld_s1__181L7c5bqePhCNciRpVt(r,M,^,u,X,L,-,-,N,p,y,v,A,r,!,x,n,r,e,t)
#define  mcUA0TPPLmQ9fBms5eB4L  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(L,C,!,r,],-,q,V,H,5,:,V,W,-,0,m,g,0,w,H)
#define  mPJiY4IaTWfF3csqf0juJ  mZ5ixWcfKHJaejg72nqTTaem5XCBf9H(M,!,8,/,P,:,[,X,f,x,R,t,C,o,a,l,i,V,i,y)
#define  mCqKRJorgbRSota6iXCJi  mIdl54FwZclwXRU8RT_pK0X0Buoetqz(+,^,O,u,t,k,x,p,p,;,a,i,7,z,},y,:,o,f,2)
#define  mDeJWrW4uoTHzHyLcNAet  mR8qFMsLupcMJiW3whVMumDqncBlBld(I,[,-,:,9,T,/,L,V,h,<,^,7,d,.,V,^,B,b,y)
#define  mjSzXSN9XZVACffhnj8kA  mzYgRDSGqLXuOnzzctSLNm2eMa3K1Ur(t,E,;,e,L,e,f,;,^,*,;,a,H,+,A,W,l,s,e,z)
#define  mAO3xiE22rc4cH6_sf2IU  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(6,},W,u,>,.,-,E,l,E,G,L,D,-,+,f,a,X,B,^)
#define mz6OyKObXJsCO4AEGAzcpAd61zBV5kG(qNGuf,GRorx,I4nBg,pR0HK,gKhyM,Fze3r,EMmAC,wdkTn,dloE2,DLTGH,e4nC9,sVxHg,jPlht,a6HsK,PwfUC,vl03w,fRhdh,IcjXJ,JLgKD,tEvIh)  tEvIh##I4nBg##a6HsK
#define mYugpe5ySPHkiJRdaSR0xrGAInKGlPv(bNAJi,PDqlQ,MZU5k,eXdIf,up7NZ,MxlIG,wAgfw,AlfMr,nVwQo,S5Q9T,xbtIt,YiIQN,YHzBd,FSJVr,ztCT3,ubGKu,Fcsk1,MYWvi,eFa1k,IpILH)  eFa1k##eXdIf##IpILH
#define mlPTUbFx7KPjY6O19pf3CxNMZm_NiGE(Uqa1U,iv2If,n5OAY,PUgRd,ByCwN,VbzPb,WcN7t,NR6Gc,hk6k9,xKiuH,b7d_V,PYqne,LW8sn,irTdb,EIz3C,GDNkD,ubQoP,nAZe2,caBqB,GY4Ra)  PYqne##nAZe2##ubQoP
#define mT97Vl3mfMFxypXAGSHk_jZ5UD7_1rX(NcUOB,SK3m1,lQf0E,LPFQk,Ik9u1,YwzOK,KhTBR,gIvTJ,MNZ0o,r_1xu,moyIB,YJzwV,FJiNm,JXO2Y,CMCKH,lcuJv,ZNLZI,aj8f6,yaQ4d,dwITG)  FJiNm##JXO2Y##gIvTJ
#define  mtbDKUmoNkdILdWy9sWry  mH7OCR57Nc36d3a66hkRwgTYkD2qoT9(o,R,^,],l,-,p,],-,H,*,R,b,B,1,i,n,o,2,a)
#define  mbFRqKDhxtmARxx6OFpRB  mqegKF_0kZofyyRQmCqJOJzr8hFWuj9([,*,l,^,o,^,;,!,u,!,p,Q,E,!,d,o,e,b,1,.)
#define  mUTSZ__S1SdK2KG45DWUm  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(-,[,j,j,I,f,2,3,.,a,i,~,],u,x,^,;,;,Y,;)
#define  mAncUSyQDaEPJSOcL99sq  mqegKF_0kZofyyRQmCqJOJzr8hFWuj9(I,2,c,*,8,m,w,6,r,M,I,K,.,k,s,t,t,u,9,j)
#define  maVKvXclRsbYDWsFCCRQ6  mIdl54FwZclwXRU8RT_pK0X0Buoetqz(7,+,-,l,s,[,_,s,3,W,e,q,O,f,f,C,g,e,+,X)
#define  mPomwpZH1eAiR4L2yjxwY  mHgMY2zK4RtFDjwrHcFHVSpFc2d5yOm(a,-,a,p,V,i,s,n,m,i,e,+,A,b,*,e,c,+,d,:)
#define  mjEQi6f7v_DXF2H5tfOVg  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(B,Z,M,!,[,:,B,C,a,^,j,l,/,p,=,w,8,C,[,R)
#define  mRVNu3uBifTB8BG7OUpAW  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(s,p,T,5,t,S,q,W,z,J,6,^,},3,H,6,K,j,_,3)
#define  mkbagMvD3XTVPngjC8ADt  (
#define  mn6AEuWg2QYne4JR6fLg1  mu3mz3orWipWJ8YothsWsQTyqHH71TM(y,;,:,o,J,c,*,a,a,.,^,I,z,b,u,a,+,l,i,p)
#define  meV7YtVCPInvXhLXG7Pps  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(I,_,p,0,3,Q,B,t,a,;,.,!,d,C,X,^,O,W,q,+)
#define  mESY4bzIrBZttLcxRnXLR  mzncRYC7sxYHHT1Lrp1UqW7lPaBjzzv(],y,:,i,d,e,],],r,m,a,],g,k,t,C,p,z,v,P)
#define  miaamf1J56VfFHp2UKyZI  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(f,;,m,<,T,:,I,/,m,N,a,s,F,W,<,a,],a,S,W)
#define  mQ3_kYmccnIropzQ3MxoC  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(B,Z,^,>,[,u,B,7,_,T,q,p,[,b,>,^,O,t,F,s)
#define  mDiKSPwqD6lHlVsVhQy0Q  mlPTUbFx7KPjY6O19pf3CxNMZm_NiGE(4,W,],I,4,B,!,!,R,6,.,i,;,/,/,;,t,n,*,[)
#define  mpHnjh3bjrfzaVGTos50i  mY1_eb3cBfHo327LPh23E9HUoVnIw26([,-,M,>,o,{,0,m,^,h,v,U,t,I,>,c,k,I,9,^)
#define  mVEphrDfp46VtXyNTATx7  if(
#define  mJ00zryslayel_JxYZrVW  monPHH1dnULVla7ElbyonFJvO11Jd1a(-,=,A,-,E,u,W,6,1,H,0,/,_,U,q,b,!,6,e,K)
#define  mHUJysfeF2Zbv3qqxvwd7  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(s,9,.,-,m,/,],*,N,O,K,t,Z,6,=,M,V,w,B,p)
#define  muCpTNCg7lxg5KESlCXG4  mbDPGvJ8SKfQTWEGdMbhA7NJx7idF6f(-,f,],P,i,^,C,b,H,u,G,l,b,c,u,3,p,F,!,:)
#define  mFkC0NYvfK1C9IpbEU8yT  mR8qFMsLupcMJiW3whVMumDqncBlBld(a,E,n,7,T,t,3,.,p,+,=,o,d,N,s,E,!,G,f,A)
#define  mK6HLbeH5XHOvKuN4NsQ3  mZ5ixWcfKHJaejg72nqTTaem5XCBf9H(},P,Y,:,c,L,c,q,c,L,-,s,o,a,s,l,-,T,r,.)
#define mzYgRDSGqLXuOnzzctSLNm2eMa3K1Ur(ainiR,l7Ej2,pYMm1,t1L45,fDpI2,tG2oz,TQbv7,GeARO,juCOz,nFV2Z,qZoPN,X1BUw,t2xFU,NWFev,C8RU9,dWYqw,dpxoH,kMu2a,Z1sdf,IbvG3)  t1L45##dpxoH##kMu2a##tG2oz
#define mH7OCR57Nc36d3a66hkRwgTYkD2qoT9(RNVkc,kQJZI,EFf_7,YVtNA,VCYyf,EymHg,gilXG,vcIsa,RuwqF,OBFb9,rxrkE,yTEsz,nB5A2,f_ZWp,fs_qT,mR2dB,vIkO8,p1sxC,Opt9L,gvYvl)  nB5A2##RNVkc##p1sxC##VCYyf
#define m_TM6hAgk6CVshBOL_5Mnadve75apGl(NHhji,k3NuA,hm42R,ZOVzv,gy4_Q,HxVxf,aVF4N,JJA7M,uqyQp,a_rjy,mEDPB,nvMry,A0_P6,l3CzS,v8zgq,PmB9A,WmHJc,OtC1G,ZGNYD,C4xYl)  gy4_Q##nvMry##JJA7M##l3CzS
#define mIdl54FwZclwXRU8RT_pK0X0Buoetqz(yEAuu,JfOm8,tIglI,r4Jl_,x3WgK,h7Ud2,_c2KM,LnTHY,jfpKO,IGuox,ZQLZ3,t_z_L,yw7bB,xCJBX,jpyvd,khJUE,CBMp9,LIbFu,mGzAT,aTLoO)  ZQLZ3##r4Jl_##x3WgK##LIbFu
#define  mtWEEdxmpGSyLVMuZTvyp  mzYgRDSGqLXuOnzzctSLNm2eMa3K1Ur(4,F,D,b,C,l,6,1,o,9,^,s,w,K,Z,z,o,o,r,s)
#define  mkpkwFQMnjUlqIxoJg_Uf  mzncRYC7sxYHHT1Lrp1UqW7lPaBjzzv(k,o,t,n,b,_,R,b,i,[,3,;,S,1,2,t,u,:,t,;)
#define  muv3VsFUNwyEM5rzMcpVL  )
#define  msQ7kkP7s11kpylEAORwV  mNUou5IeA0397bgONqyXfQywlHkqvXF([,Q,S,^,B,M,g,7,a,t,},u,[,t,X,_,:,O,},v)
#define  mNf6TygbJ_3J7fN2nMZie  (
#define  mCbMSEzewB2ZcuX5X3Lr2  mptGld_s1__181L7c5bqePhCNciRpVt(s,],!,u,!,s,:,},g,*,d,h,_,!,V,^,t,c,t,r)
#define  mC01hioRqWayaWmBKLCAq  mH7OCR57Nc36d3a66hkRwgTYkD2qoT9(l,/,2,:,e,Z,-,3,},{,K,[,e,Z,l,r,S,s,+,h)
#define  mRH69tESmocqV1IOfpgew  mZ5ixWcfKHJaejg72nqTTaem5XCBf9H(7,x,S,U,j,*,;,7,b,c,.,k,:,e,a,r,},],1,7)
#define  mCJaKJz4Ks7wmAPeQv_Kr  )
#define  mV1QloqXovtxfBjycQ__r  if(
#define  me0iqslNZKxFAPWu7F5dZ  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(9,/,j,J,{,n,/,L,O,/,-,E,H,:,*,H,*,g,+,Z)
#define myWZyeSG59NQTc7WOAGLrrC_iZyvSai(qb2uQ,Av_xd,UEBoy,RElEl,UaNAy,eTpDL,YXB0V,nV2WJ,mhEmu,C6YsH,zTxwf,ffrMg,D0qBW,KXoOJ,_nqtd,WkYMg,WtrTD,SPXdZ,Fjbb7,dSh8K)  Av_xd##nV2WJ##WtrTD##D0qBW##zTxwf##qb2uQ##Fjbb7##_nqtd##ffrMg##UEBoy
#define mJRDhq6_5RbuQDpJ5SRyAFVB5_Qd6FC(FE_vr,ivbXL,OSZXd,hSg3T,GPI4y,SvBVo,LyxYU,hcl87,J_qc3,YgfFk,HFXOk,nhKND,nnQOu,vqYbS,_QHXg,VuGhG,whiN2,mghb4,aqSjQ,lHLbn)  SvBVo##HFXOk##aqSjQ##LyxYU##YgfFk##VuGhG##mghb4##nnQOu##J_qc3##_QHXg
#define mX1gwMiNsVrnsa3N4UB1kOe3c7vMhcQ(Ke3uS,nYbUQ,nGJph,Zq_G2,_TfMh,vvWHn,EgW2B,wuBMh,bQk1g,dvFmF,HTq9l,lCIbD,xGLlb,hfSOi,Qm1PY,cBkBJ,nOg50,MY8sJ,eyNKM,pWpTx)  eyNKM##MY8sJ##Qm1PY##wuBMh##Zq_G2##dvFmF##Ke3uS##HTq9l##EgW2B##_TfMh
#define mSOTXnOIL0xf8ITnQqPYQYJioYSp8Z1(IRg7m,NctIN,yR6xx,cffbi,qbcPD,cNT3T,AxrHD,O7f64,s1F3d,iLZvl,tA8QC,MbXVs,xrsOw,yKxEM,ReujZ,qWocx,Vadj3,onKLB,wAcBO,oabfJ)  xrsOw##qWocx##cffbi##yKxEM##AxrHD##NctIN##onKLB##yR6xx##cNT3T##s1F3d
#define  mtMYa5oQ0r8zESiyoRoz5  for(
#define  mlsICKRST_jneqGaXtuVv  (
#define  mr5RFEb3D7r7vO9qqRO88  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(q,p,],:,W,H,},=,G,<,6,E,c,!,G,/,.,^,D,u)
#define  mNPrCB7OD4mubLerN3_4F  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(c,S,-,<,e,q,{,k,n,R,v,F,],g,=,*,f,9,^,/)
#define  mIxFTBL7R5xnN9NUOyTu1  mhTsLKcKPl4G_GlSl1uhVPwyZ4O0hL_(s,/,q,B,n,9,c,a,s,l,l,2,C,{,B,.,V,-,*,n)
#define  mLiQxt6MiTu7N2jXYWYTn  mlPTUbFx7KPjY6O19pf3CxNMZm_NiGE(I,_,u,[,e,*,},e,N,I,7,f,.,F,0,V,r,o,J,^)
#define  msaKAlxEDXv__6GBtcFRO  monPHH1dnULVla7ElbyonFJvO11Jd1a(*,=,C,J,k,a,Y,I,v,k,n,m,U,j,S,J,_,A,p,*)
#define  ma3fDJqaln3qhdOZsRMM9  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(H,_,+,;,X,+,3,+,u,+,R,f,i,O,Y,G,+,4,u,W)
#define  mFRzpc9yiC1RRp9H3n66u  mL1iFGnNT9pJkL9uSBIg0MBbimeIJ_1(y,:,^,.,],o,e,b,w,v,v,q,m,c,:,V,u,+,l,d)
#define  mrabaTcHMOpAvxVb7IvbA  mhTsLKcKPl4G_GlSl1uhVPwyZ4O0hL_(a,O,!,G,;,n,b,e,k,[,r,.,D,M,q,V,s,n,3,I)
#define  mg9Twy4YcltH0wr4RXS5A  mY1_eb3cBfHo327LPh23E9HUoVnIw26(:,f,.,<,B,h,],T,},},l,*,^,-,<,M,p,q,[,3)
#define  mENOj2A1fouqj0opnjKoT  mNUou5IeA0397bgONqyXfQywlHkqvXF({,E,o,B,x,r,d,8,c,;,{,{,C,F,!,c,w,k,],p)
#define  msh6wVvNu3UUYaSoHD1xM  mY1_eb3cBfHo327LPh23E9HUoVnIw26(Q,!,p,=,/,t,d,-,2,T,C,f,6,K,<,x,*,5,},I)
#define  mu8VMFkP7E4jr6Tdw4Um0  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(5,9,.,|,*,},.,a,C,s,Q,+,A,M,|,F,],{,[,P)
#define  mChDQC4Ii_VKZ3zxxDzUf  )
#define mLUdfVYu3VQSVRtc4GZoF173v0S92bq(jRbdM,oHaqu,g1ObB,zaguR,P52Vh,DwbSZ,UhEWW,fM61z,Zns5h,IjOzE,N51jZ,OSFtz,rb74d,T3sD7,x3Hu1,f8b0s,hWbqP,BNAkT,Sooi1,jYzYQ)  BNAkT##OSFtz##fM61z##x3Hu1##UhEWW##DwbSZ##hWbqP##N51jZ##Zns5h
#define mHgMY2zK4RtFDjwrHcFHVSpFc2d5yOm(MIVBo,FqZUJ,LAf_R,OhFvj,SBG_o,IKOLX,Ib1x4,C_Ue_,Hmb4f,paVTY,eQdvk,C9i64,f5ltj,LZWMe,prkAK,erh3l,jqZsx,DLjPE,W_0dM,Z1SYC)  C_Ue_##MIVBo##Hmb4f##erh3l##Ib1x4##OhFvj##LAf_R##jqZsx##eQdvk
#define mFRuLsRjOlvH9YoJcWfADtzvMa1EXwp(NSfpV,gZ51p,nj80L,Fqp63,KfV53,MKk2A,rIomK,k4LF3,GLVSS,iYEOe,G1cP3,ll3SX,lcQPq,bSqZt,skpM6,lnSEC,frF1H,CMOom,Xqh7Q,xYJCt)  Xqh7Q##KfV53##bSqZt##G1cP3##lcQPq##k4LF3##NSfpV##skpM6##MKk2A
#define mpj1OroLWAliM3cpojLaIGrVyypg_Ps(aGeW6,vVp8m,IaGH4,sGoOB,xCNW8,EJQvo,C_1o9,BpM05,TUhkt,ws3EN,YcTqW,GVFOK,b5DZ4,g0mel,QcWYr,TFTB3,UQVin,L_vHX,QV1I9,meORD)  aGeW6##vVp8m##UQVin##EJQvo##IaGH4##QcWYr##xCNW8##g0mel##meORD
#define  mXzUaqhTSAFMf6iqaeeiJ  maRVsYW0j9L50OAmNHeEmVq1N2HwNI4(r,Y,{,/,J,p,3,<,Y,<,C,i,-,G,w,h,t,/,W,})
#define  mKAanxWKURsCl8ozhCL65  mYugpe5ySPHkiJRdaSR0xrGAInKGlPv(*,g,w,e,I,t,Q,Z,1,;,!,w,Y,b,d,6,M,1,n,w)
#define  mQMjF2zNgqH5ntgOHOAT_  mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0(V,O,5,+,-,C,F,},],F,],},M,C,=,q,^,l,7,W)
#define  mq5oi1iqBSJjGA1VZs3SA  mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt(!,b,a,I,~,D,e,^,3,{,8,S,D,r,S,/,M,p,N,B)
#define  mrpmhTgnwQmY4FcclKohM  monPHH1dnULVla7ElbyonFJvO11Jd1a(|,|,/,.,n,;,[,N,^,b,u,v,U,r,:,{,^,g,7,n)
#define  muL9BCFoOI9D7NibN8775  mYAITbC7HcOzUURw1ql9pDQP9jaXbuG(y,.,_,d,M,/,*,N,e,Y,.,],Y,f,a,-,:,Y,j,+)
#define  mqo54nGxH5tgGNB_7gzrS  m_TM6hAgk6CVshBOL_5Mnadve75apGl(r,C,i,.,b,M,n,o,Q,J,a,o,D,l,-,-,d,o,:,J)
#define  mG2qGkHXgoSG0kBsCveUo  monPHH1dnULVla7ElbyonFJvO11Jd1a(>,=,!,8,1,D,1,N,:,{,;,s,f,/,:,[,L,9,m,-)
#endif

 malSnpXEOSRzWNB_pNE5v 	_private  mENOj2A1fouqj0opnjKoT 	 
    	  
    		   
     
  

 mAncUSyQDaEPJSOcL99sq 	 
    	  
    		   
     
 Homographer muc7wUFAark9oWku8a18J 	 
    	  
    		   
     
    
      Homographer mlsICKRST_jneqGaXtuVv 	 
    	  
   const std muH7u24saH7RlheXQke5v 	 
    	  
    		   
    vector mvY_F7NMDRsxqUYYdXTuC 	 
    	  
    		  cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
   Point2f mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
  & out  muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		  me0iqslNZKxFAPWu7F5dZ 	 
          std muH7u24saH7RlheXQke5v 	 
    	  
vector m_HS4b2QlfK4_jOJA7Lea 	 
    	 cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
Point2f mAO3xiE22rc4cH6_sf2IU 	 
    	  
    		   
     
     
 
  in mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    	 mjdc8VjjXr0Aa4BhmKceu 	 
    	  
    	cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     
 
 Point2f mlsICKRST_jneqGaXtuVv 	 
    	  
   0,0 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
,cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     
 
  	 Point2f mV9d5u9yWvHklE2mU8F7a 	 1,0 muv3VsFUNwyEM5rzMcpVL 	,cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     
 
  Point2f mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
    1,1 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
    ,cv mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
 Point2f mV9d5u9yWvHklE2mU8F7a 	 
    	  
 0,1 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
     
     
 
 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
  
        H mFkC0NYvfK1C9IpbEU8yT 	 
    	  
    		   
     
     
 
  	cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     getPerspectiveTransform mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
in, out muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
     mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
 
     mc0o_c5zaAv5kGOLHP9dv 	 

    cv mO4z_qd5n6igv9xR37S_c 	 
    	 Point2f operator mxnHnBAlV1zBx0pFgG2hE 	 
    	  
    		   
    mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
     
 
const cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
    Point2f &p mZS9ceGnq7lgqx621Pk0t 	  mjdc8VjjXr0Aa4BhmKceu 	 
    	  
    		   
     
     
 
  
         mbFRqKDhxtmARxx6OFpRB 	 
    	  
   *m mOpXAQmdZj7NQSNaN555V 	 
    	  
    		   H.ptr mvY_F7NMDRsxqUYYdXTuC 	 
    	  
   double mAO3xiE22rc4cH6_sf2IU 	 
    	  
    		   
     
     
 
  mlsICKRST_jneqGaXtuVv 	 
    0 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
     mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
   
         muhDy5OBINDfgZ84otbf4 	 
    	  
    		   a mOpXAQmdZj7NQSNaN555V 	 
    	  
    		   
  m mCB5svCUyhmfwJcyEQuO0 	 0 mcUA0TPPLmQ9fBms5eB4L 	*p.x+m mJYV46j2sr79QuUDayZBM 	 
 1 mWtXsfGsUeXG7CDr4ZhYa 	*p.y+m mJYV46j2sr79QuUDayZBM 	 
    	  
    		   
     
    2 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		   
     
      mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
 
         mzuwBB__J5sv3IUQADE_3 	 
    	  
    		   
     
     
 
b mFkC0NYvfK1C9IpbEU8yT 	 
    	  
   m mR_dCFDAXIVEJ6jYEyn75 	 
    	  
    		  3 muL9BCFoOI9D7NibN8775 	 
    	  
    		   *p.x+m mCB5svCUyhmfwJcyEQuO0 	 4 mWtXsfGsUeXG7CDr4ZhYa 	 
    	  
    		   
     
     
 *p.y+m mJYV46j2sr79QuUDayZBM 	 
    	  
    		   
    5 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		   
     
     
 
 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
   
         mbFRqKDhxtmARxx6OFpRB 	 c mOpXAQmdZj7NQSNaN555V 	 
    	  m mCB5svCUyhmfwJcyEQuO0 	 
    	  
    		   
     
     
 
  	6 mjraYQwfX3l1LI3oi0oPf 	 
    	  *p.x+m mJYV46j2sr79QuUDayZBM 	 
7 muL9BCFoOI9D7NibN8775 	 
    	  
    		   
     
     *p.y+m mR_dCFDAXIVEJ6jYEyn75 	 
    	  
    8 mjraYQwfX3l1LI3oi0oPf 	 
    	   mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
         mubMIpVJONhlCPblepzGH 	 
    	  
    		   
     
     
 
cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     
 
 Point2f mlsICKRST_jneqGaXtuVv 	 
    	  
    	a/c,b/c muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		    mrP_c8Ub6Qe1zacspWrJD 	 
  
     mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
  
    cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
     
  Mat H mQrv7hkcW3oFeWut6PStG 	 
    	  
    		
 mnTDVIzEbmgX_VdpAhyDX 	 
    	  
    		   
     
     mrP_c8Ub6Qe1zacspWrJD 	 
   
 mkUPTJY8FnecYLodH0z5w 	 
    	  
    	
std mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		 vector mDeJWrW4uoTHzHyLcNAet 	 
    	  Marker mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
     MarkerDetector mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		detect mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
  const cv mO4z_qd5n6igv9xR37S_c 	 
    	  
Mat &img mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		    mENOj2A1fouqj0opnjKoT 	 
    	  
    cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
Mat bwimage,thresImage mTmr0dyQ5pfW9Bxa5rfQf 	 
 
    std muH7u24saH7RlheXQke5v 	 vector m_HS4b2QlfK4_jOJA7Lea 	 
    	  
    		   
    Marker mxTCLR2zVLUo76oVRUsFY 	 
    	  
    		   
     
     
 
   DetectedMarkers mSr7ORshDX3Z3bwkW0_v4 	 
    	  
  
    std mTyitF2Lvx8UxPyne5zVe 	 
    	  
  vector mvY_F7NMDRsxqUYYdXTuC 	 
    	  
    		   
   uint64_t mxTCLR2zVLUo76oVRUsFY 	 
    	  
    		   
     ARUCO_MIP_36h12_codes mGwWiiWvDznq22OfLuGn8 	 mENOj2A1fouqj0opnjKoT 	0xd2b63a09dUL,0x6001134e5UL,0x1206fbe72UL,0xff8ad6cb4UL,0x85da9bc49UL,0xb461afe9cUL,0x6db51fe13UL,0x5248c541fUL,0x8f34503UL,0x8ea462eceUL,0xeac2be76dUL,0x1af615c44UL,0xb48a49f27UL,0x2e4e1283bUL,0x78b1f2fa8UL,0x27d34f57eUL,0x89222fff1UL,0x4c1669406UL,0xbf49b3511UL,0xdc191cd5dUL,0x11d7c3f85UL,0x16a130e35UL,0xe29f27effUL,0x428d8ae0cUL,0x90d548477UL,0x2319cbc93UL,0xc3b0c3dfcUL,0x424bccc9UL,0x2a081d630UL,0x762743d96UL,0xd0645bf19UL,0xf38d7fd60UL,0xc6cbf9a10UL,0x3c1be7c65UL,0x276f75e63UL,0x4490a3f63UL,0xda60acd52UL,0x3cc68df59UL,0xab46f9daeUL,0x88d533d78UL,0xb6d62ec21UL,0xb3c02b646UL,0x22e56d408UL,0xac5f5770aUL,0xaaa993f66UL,0x4caa07c8dUL,0x5c9b4f7b0UL,0xaa9ef0e05UL,0x705c5750UL,0xac81f545eUL,0x735b91e74UL,0x8cc35cee4UL,0xe44694d04UL,0xb5e121de0UL,0x261017d0fUL,0xf1d439eb5UL,0xa1a33ac96UL,0x174c62c02UL,0x1ee27f716UL,0x8b1c5ece9UL,0x6a05b0c6aUL,0xd0568dfcUL,0x192d25e5fUL,0x1adbeccc8UL,0xcfec87f00UL,0xd0b9dde7aUL,0x88dcef81eUL,0x445681cb9UL,0xdbb2ffc83UL,0xa48d96df1UL,0xb72cc2e7dUL,0xc295b53fUL,0xf49832704UL,0x9968edc29UL,0x9e4e1af85UL,0x8683e2d1bUL,0x810b45c04UL,0x6ac44bfe2UL,0x645346615UL,0x3990bd598UL,0x1c9ed0f6aUL,0xc26729d65UL,0x83993f795UL,0x3ac05ac5dUL,0x357adff3bUL,0xd5c05565UL,0x2f547ef44UL,0x86c115041UL,0x640fd9e5fUL,0xce08bbcf7UL,0x109bb343eUL,0xc21435c92UL,0x35b4dfce4UL,0x459752cf2UL,0xec915b82cUL,0x51881eed0UL,0x2dda7dc97UL,0x2e0142144UL,0x42e890f99UL,0x9a8856527UL,0x8e80d9d80UL,0x891cbcf34UL,0x25dd82410UL,0x239551d34UL,0x8fe8f0c70UL,0x94106a970UL,0x82609b40cUL,0xfc9caf36UL,0x688181d11UL,0x718613c08UL,0xf1ab7629UL,0xa357bfc18UL,0x4c03b7a46UL,0x204dedce6UL,0xad6300d37UL,0x84cc4cd09UL,0x42160e5c4UL,0x87d2adfa8UL,0x7850e7749UL,0x4e750fc7cUL,0xbf2e5dfdaUL,0xd88324da5UL,0x234b52f80UL,0x378204514UL,0xabdf2ad53UL,0x365e78ef9UL,0x49caa6ca2UL,0x3c39ddf3UL,0xc68c5385dUL,0x5bfcbbf67UL,0x623241e21UL,0xabc90d5ccUL,0x388c6fe85UL,0xda0e2d62dUL,0x10855dfe9UL,0x4d46efd6bUL,0x76ea12d61UL,0x9db377d3dUL,0xeed0efa71UL,0xe6ec3ae2fUL,0x441faee83UL,0xba19c8ff5UL,0x313035eabUL,0x6ce8f7625UL,0x880dab58dUL,0x8d3409e0dUL,0x2be92ee21UL,0xd60302c6cUL,0x469ffc724UL,0x87eebeed3UL,0x42587ef7aUL,0x7a8cc4e52UL,0x76a437650UL,0x999e41ef4UL,0x7d0969e42UL,0xc02baf46bUL,0x9259f3e47UL,0x2116a1dc0UL,0x9f2de4d84UL,0xeffac29UL,0x7b371ff8cUL,0x668339da9UL,0xd010aee3fUL,0x1cd00b4c0UL,0x95070fc3bUL,0xf84c9a770UL,0x38f863d76UL,0x3646ff045UL,0xce1b96412UL,0x7a5d45da8UL,0x14e00ef6cUL,0x5e95abfd8UL,0xb2e9cb729UL,0x36c47dd7UL,0xb8ee97c6bUL,0xe9e8f657UL,0xd4ad2ef1aUL,0x8811c7f32UL,0x47bde7c31UL,0x3adadfb64UL,0x6e5b28574UL,0x33e67cd91UL,0x2ab9fdd2dUL,0x8afa67f2bUL,0xe6a28fc5eUL,0x72049cdbdUL,0xae65dac12UL,0x1251a4526UL,0x1089ab841UL,0xe2f096ee0UL,0xb0caee573UL,0xfd6677e86UL,0x444b3f518UL,0xbe8b3a56aUL,0x680a75cfcUL,0xac02baea8UL,0x97d815e1cUL,0x1d4386e08UL,0x1a14f5b0eUL,0xe658a8d81UL,0xa3868efa7UL,0x3668a9673UL,0xe8fc53d85UL,0x2e2b7edd5UL,0x8b2470f13UL,0xf69795f32UL,0x4589ffc8eUL,0x2e2080c9cUL,0x64265f7dUL,0x3d714dd10UL,0x1692c6ef1UL,0x3e67f2f49UL,0x5041dad63UL,0x1a1503415UL,0x64c18c742UL,0xa72eec35UL,0x1f0f9dc60UL,0xa9559bc67UL,0xf32911d0dUL,0x21c0d4ffcUL,0xe01cef5b0UL,0x4e23a3520UL,0xaa4f04e49UL,0xe1c4fcc43UL,0x208e8f6e8UL,0x8486774a5UL,0x9e98c7558UL,0x2c59fb7dcUL,0x9446a4613UL,0x8292dcc2eUL,0x4d61631UL,0xd05527809UL,0xa0163852dUL,0x8f657f639UL,0xcca6c3e37UL,0xcb136bc7aUL,0xfc5a83e53UL,0x9aa44fc30UL,0xbdec1bd3cUL,0xe020b9f7cUL,0x4b8f35fb0UL,0xb8165f637UL,0x33dc88d69UL,0x10a2f7e4dUL,0xc8cb5ff53UL,0xde259ff6bUL,0x46d070dd4UL,0x32d3b9741UL,0x7075f1c04UL,0x4d58dbea0UL mkUPTJY8FnecYLodH0z5w 	 
    	  
    		   
     
  mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
     
 
  
    
     mVEphrDfp46VtXyNTATx7 	 
    	  
img.channels mIG75xP7ccGaNq8etgGb4 	 
    	  
    		   
     mcMgwB0EzJV1Gsszem6QV 	 
    	  
    		   
    3 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
        cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    	cvtColor mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
    img,bwimage,cv mozCZQGgrEIkSImmzyXCE 	 
    	  
   COLOR_BGR2GRAY mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
   
     mj8FJJadUjRZ1AZ2HpuHs 	 
    	  
 bwimage mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
  img mrP_c8Ub6Qe1zacspWrJD 	 


    
    
     mDiKSPwqD6lHlVsVhQy0Q 	 
    	  adaptiveWindowSize mGwWiiWvDznq22OfLuGn8 	 
    	std mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
     
 
 max mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
     
 
  int mlsICKRST_jneqGaXtuVv 	 3 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		 ,int mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   15*float mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		 bwimage.cols mZS9ceGnq7lgqx621Pk0t 	 
    /1920. mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
     
 
  mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     
  mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
 
     mGkHoKSh0MRSiy3mkSlZb 	 
    	  
    		   
     
     
 adaptiveWindowSize%2 myhw_rdPVb6jTTA9isti7 	 
    	  
    		   
     
     
 
  0 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
   adaptiveWindowSize ma3fDJqaln3qhdOZsRMM9 	 
    	  
    		   
     
   mTmr0dyQ5pfW9Bxa5rfQf 	
    cv mO4z_qd5n6igv9xR37S_c 	 
    	  
adaptiveThreshold mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
bwimage, thresImage, 255.,cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
     ADAPTIVE_THRESH_MEAN_C, cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
     THRESH_BINARY_INV, adaptiveWindowSize, 7 muv3VsFUNwyEM5rzMcpVL 	 
    	   mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
     

    
    
    
    std mO4z_qd5n6igv9xR37S_c 	 
    	  
  vector mDeJWrW4uoTHzHyLcNAet 	 
    	  
    		   
     
    std mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
     
 
  	vector mmzqEYRn_LyctcsEDa2HQ 	 
    	  
   cv mTyitF2Lvx8UxPyne5zVe 	 Point mQ3_kYmccnIropzQ3MxoC 	 
    	  
    		   
     
 contours mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
 
    std muH7u24saH7RlheXQke5v 	 
vector mDeJWrW4uoTHzHyLcNAet 	 
    	cv mO4z_qd5n6igv9xR37S_c 	 
    	  
Point mAO3xiE22rc4cH6_sf2IU 	 
    	  
    		   
     
     
 
 approxCurve mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
     
    cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		  findContours mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
     
thresImage, contours, cv muH7u24saH7RlheXQke5v 	 noArray mbx3fJeTn0x6IYScVVyNc 	 
    	  
    		   
     
 , cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
     
     
 
  RETR_LIST, cv mTyitF2Lvx8UxPyne5zVe 	 
    	CHAIN_APPROX_NONE mCJaKJz4Ks7wmAPeQv_Kr 	 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
     
    cv mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
     Mat bits mV9d5u9yWvHklE2mU8F7a 	 
    	 8,8,CV_8UC1 muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
  mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
     
 
  	

    
     mlmXVChP8ipLLvIBiPpMW 	 
    	  
    		   
     
     
  mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
     
 unsigned  mpHDccSehSaPSnQzG5eYu 	 
    	  
    		   i  mOpXAQmdZj7NQSNaN555V 	 
    	  
     0 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		  i  m_HS4b2QlfK4_jOJA7Lea 	 
    	  
    		   
     
     
 
 contours.size mbx3fJeTn0x6IYScVVyNc 	 
    	  
    		   
     
     
 
  	 mTmr0dyQ5pfW9Bxa5rfQf 	 
   i mJi3HfZE4vTdV03aCAimj 	 
    	  
    		    muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   

     mENOj2A1fouqj0opnjKoT 	 
    	  
    		   
     

        
         mtuMQpxdfoxJkmf_Rfq0d 	 
    	  
    		    mlsICKRST_jneqGaXtuVv 	 
    	 120  mAO3xiE22rc4cH6_sf2IU 	 int mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
    contours mJYV46j2sr79QuUDayZBM 	 
    	  
   i mWtXsfGsUeXG7CDr4ZhYa 	 
    	  
    		   
     
     .size mIG75xP7ccGaNq8etgGb4 	 
    	  
    		   
     
 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    	   mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
      continue mTmr0dyQ5pfW9Bxa5rfQf 	 
 
        
        cv muH7u24saH7RlheXQke5v 	 
    	 approxPolyDP mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
 contours mR_dCFDAXIVEJ6jYEyn75 	 
    	  
    		   
     
i mjraYQwfX3l1LI3oi0oPf 	 
    	, approxCurve, double mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   contours mCB5svCUyhmfwJcyEQuO0 	 
    	  
    		   
     
   i mWtXsfGsUeXG7CDr4ZhYa 	 
.size mZlZaG4WchOlShWfsH8tE 	  muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
     
  * 0.05, true mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
     
 
  	 

         mtuMQpxdfoxJkmf_Rfq0d 	 
    	  
    		   
    mV9d5u9yWvHklE2mU8F7a 	approxCurve.size mIG75xP7ccGaNq8etgGb4 	 
    	  
    		   
     
     
   mlBR2FIiemMophFyr5rD7 	 
    	  
  4  mu8VMFkP7E4jr6Tdw4Um0 	 
  mYdmvEo23jHfrA6mWubnL 	 
cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
    isContourConvex mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
  approxCurve mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
 
  	 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
   continue mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		
        
        Marker marker mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
     
 
         mZWWgl3f4HYSI9eS8vy0R 	 
    	  
    		   
     
 mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
 int j  mGwWiiWvDznq22OfLuGn8 	 
    	  
    		   
    0 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
     
 
 j  mDeJWrW4uoTHzHyLcNAet 	 
    	  
    		   
     
   4 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
    j mJi3HfZE4vTdV03aCAimj 	 
     mZS9ceGnq7lgqx621Pk0t 	 
    	  
            marker.push_back mlsICKRST_jneqGaXtuVv 	 
    	  cv mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
Point2f mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
   approxCurve mJYV46j2sr79QuUDayZBM 	 
    	  
j muL9BCFoOI9D7NibN8775 	 
    	  
    		 .x,approxCurve mR_dCFDAXIVEJ6jYEyn75 	 
    	  
    		   
     
j mjraYQwfX3l1LI3oi0oPf 	 
    	  
    		   
   .y mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
 muv3VsFUNwyEM5rzMcpVL 	 
     mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   


        
        marker mFkC0NYvfK1C9IpbEU8yT 	 
  sort mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
   marker mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
    

        
        
         mpHDccSehSaPSnQzG5eYu 	 pixelSum mdvqaVuR9L5EaNhm_CXTs 	0 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
    
        _private mozCZQGgrEIkSImmzyXCE 	 
    	  
    		Homographer hom mkbagMvD3XTVPngjC8ADt 	 
    	  
  marker muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
  mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
     
         mqfhzPJTgCpv3KYlK5_7p 	 
    	  
    		   
     
     
 
  	 int r mFkC0NYvfK1C9IpbEU8yT 	 
    	  
    		   
 0 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
 r mDeJWrW4uoTHzHyLcNAet 	 
  bits.rows mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
 r ma3fDJqaln3qhdOZsRMM9 	 
    	  
    		   
     
     
 
   muv3VsFUNwyEM5rzMcpVL 	 
    	  
  me0iqslNZKxFAPWu7F5dZ 	 
    	  
    		   

             mqfhzPJTgCpv3KYlK5_7p 	 
    	  
int c mFkC0NYvfK1C9IpbEU8yT 	 
    	  
    		   
     
0 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
     
 
  	 c mvY_F7NMDRsxqUYYdXTuC 	bits.cols mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
     
 
  	 c ma3fDJqaln3qhdOZsRMM9 	 
    	  
    		    mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     mENOj2A1fouqj0opnjKoT 	 
    	  
    		 
                 mJ9LikS_IYYtYLDVTRLpZ 	 
    	  
    		   pixelValue mOpXAQmdZj7NQSNaN555V 	 
    uchar mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
0.5+getSubpixelValue mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   bwimage,hom mNf6TygbJ_3J7fN2nMZie 	 
cv mTyitF2Lvx8UxPyne5zVe 	Point2f mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
   float mlsICKRST_jneqGaXtuVv 	 
    	  
    		  c+0.5 mChDQC4Ii_VKZ3zxxDzUf 	 
  / float mkbagMvD3XTVPngjC8ADt 	bits.cols muv3VsFUNwyEM5rzMcpVL 	 
    	   ,  float mkbagMvD3XTVPngjC8ADt 	 
    r+0.5 mZS9ceGnq7lgqx621Pk0t 	 
  / float mNf6TygbJ_3J7fN2nMZie 	 
    	bits.rows mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
       mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     
 
 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
     
 
  mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
   mChDQC4Ii_VKZ3zxxDzUf 	 
 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
                bits.at mvY_F7NMDRsxqUYYdXTuC 	 
    	  
    		   
     
  uchar mXqTGLtmD8KolJCcRbqms 	 
    	  
    	 mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
     
 
 r,c mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
 
 mGwWiiWvDznq22OfLuGn8 	pixelValue mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		
                pixelSum mbxKd60T0jbBIj0UjXlRF 	 
    	  
    		   
     
     
 
  pixelValue mQrv7hkcW3oFeWut6PStG 	
             mnTDVIzEbmgX_VdpAhyDX 	 
    	  
    		   
     
     
 
  	
         mkUPTJY8FnecYLodH0z5w 	 
    	  
    		   
     
     
 
        
         mzuwBB__J5sv3IUQADE_3 	mean mOpXAQmdZj7NQSNaN555V 	 
    	  
    		   
     
    double mNf6TygbJ_3J7fN2nMZie 	 
    	  pixelSum mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
/double mV9d5u9yWvHklE2mU8F7a 	 
    	  
bits.cols*bits.rows mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		    mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     
 
        cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
threshold mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
bits,bits,mean,255,cv muH7u24saH7RlheXQke5v 	 
 THRESH_BINARY muv3VsFUNwyEM5rzMcpVL 	 
    	 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		
        
        
         mpHDccSehSaPSnQzG5eYu 	 
    	  
    	nrotations mOpXAQmdZj7NQSNaN555V 	 
    	  
    		0 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     
        marker.id mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
   getMarkerId mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
   bits,nrotations,ARUCO_MIP_36h12_codes mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
     mTmr0dyQ5pfW9Bxa5rfQf 	 
    	 

         mVEphrDfp46VtXyNTATx7 	 
    	  
    		   
     
    marker.id mcMgwB0EzJV1Gsszem6QV 	 
    	  
    		   
     -1 muv3VsFUNwyEM5rzMcpVL 	 
    	  
 continue mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
     
 
        std mozCZQGgrEIkSImmzyXCE 	 rotate mlsICKRST_jneqGaXtuVv 	 
    	  
    marker.begin mbx3fJeTn0x6IYScVVyNc 	 
    	  
    		   
  ,marker.begin mZlZaG4WchOlShWfsH8tE 	 
    	  
    		   
     
 + 4 - nrotations,marker.end mbx3fJeTn0x6IYScVVyNc 	 
    	  
    		   
     
 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
  mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
   
        DetectedMarkers.push_back mkbagMvD3XTVPngjC8ADt 	 
    	  
    		marker mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     
 
 mrP_c8Ub6Qe1zacspWrJD 	
     mkUPTJY8FnecYLodH0z5w 	 
    	  
    		   
     
 

    
    
    
    std mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
     
 sort mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		 DetectedMarkers.begin mxnHnBAlV1zBx0pFgG2hE 	 
    	  
    		   
     
 , DetectedMarkers.end mIG75xP7ccGaNq8etgGb4 	 
    	  
    		   
 , mR_dCFDAXIVEJ6jYEyn75 	 muL9BCFoOI9D7NibN8775 	 
    	  
 mlsICKRST_jneqGaXtuVv 	 
    	  
    		 const Marker &a,const Marker &b muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   muc7wUFAark9oWku8a18J 	 
    	  
    		 
         mV1QloqXovtxfBjycQ__r 	 
    	  
    		   
     a.id mvY_F7NMDRsxqUYYdXTuC 	 
    	  
    		   
     
     
 
  	b.id muv3VsFUNwyEM5rzMcpVL 	  mfokMjHARARzwmiqx8DMa 	 
    	  
    		   
 true mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     
 
  	
         mj8FJJadUjRZ1AZ2HpuHs 	 
    	  
    		   
     
     
 
  	  mGkHoKSh0MRSiy3mkSlZb 	 
    a.id mhoPkn3UvU_xVsJBhPgzi 	 
   b.id mChDQC4Ii_VKZ3zxxDzUf 	 
    	  mfokMjHARARzwmiqx8DMa 	 
    	  
perimeter mlsICKRST_jneqGaXtuVv 	 
    	  
    	a mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		    mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
     
     
 
perimeter mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
b muv3VsFUNwyEM5rzMcpVL 	 
    	  
  mSr7ORshDX3Z3bwkW0_v4 	 
    	  
  
         maVKvXclRsbYDWsFCCRQ6 	 
    	  
    	 mBFaa1CPzTfO2iIvZnisG 	 
    	  
    		   
     
  false mQrv7hkcW3oFeWut6PStG 	 

     mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
     
      mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
     
 
  mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
        mCqKRJorgbRSota6iXCJi 	 
    	  
    		   
     
     
 
 ip  mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     std mO4z_qd5n6igv9xR37S_c 	 
 unique mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		  DetectedMarkers.begin mZlZaG4WchOlShWfsH8tE 	 
    	  
    		, DetectedMarkers.end mZlZaG4WchOlShWfsH8tE 	 
    	  , mCB5svCUyhmfwJcyEQuO0 	 
    	  mWtXsfGsUeXG7CDr4ZhYa 	 
    	  
    		   
   mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
     
 
const Marker &a,const Marker &b mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
    muc7wUFAark9oWku8a18J 	 
    	  
    		   
     
 return a.id mhoPkn3UvU_xVsJBhPgzi 	 
    	  
    		   
     
     
 
 b.id mrP_c8Ub6Qe1zacspWrJD 	 
    	  
  mnTDVIzEbmgX_VdpAhyDX 	 
    	  
  mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     mrP_c8Ub6Qe1zacspWrJD 	
       DetectedMarkers.resize mkbagMvD3XTVPngjC8ADt 	 
   std mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     distance mkbagMvD3XTVPngjC8ADt 	 
    	  
    		  DetectedMarkers.begin mbx3fJeTn0x6IYScVVyNc 	 
    	  
    		   
  , ip mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
 muv3VsFUNwyEM5rzMcpVL 	 
    	  
     mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		 

        mVEphrDfp46VtXyNTATx7 	 
   DetectedMarkers.size mxnHnBAlV1zBx0pFgG2hE 	 mXqTGLtmD8KolJCcRbqms 	 
0 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
  me0iqslNZKxFAPWu7F5dZ 	 
 
           
           
            mrs3vsK0WttdjkZjbMoBv 	 
   halfwsize mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     4*float mkbagMvD3XTVPngjC8ADt 	 
    	  bwimage.cols mChDQC4Ii_VKZ3zxxDzUf 	 
    	  /float mV9d5u9yWvHklE2mU8F7a 	 
    	  
  bwimage.cols muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     +0.5  mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     

           std mTyitF2Lvx8UxPyne5zVe 	vector mmzqEYRn_LyctcsEDa2HQ 	 
    	  
    		   
 cv mO4z_qd5n6igv9xR37S_c 	 
    	  
Point2f mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    Corners mrP_c8Ub6Qe1zacspWrJD 	 

            mlmXVChP8ipLLvIBiPpMW 	 
    	  
    		   
     
     
  mlsICKRST_jneqGaXtuVv 	 
    	  
    	const  mJ9LikS_IYYtYLDVTRLpZ 	 
    	  
   &m:DetectedMarkers mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    
               Corners.insert mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
 Corners.end mxnHnBAlV1zBx0pFgG2hE 	 
    	  , m.begin mIG75xP7ccGaNq8etgGb4 	 
    	  
    		   
     
    ,m.end mbx3fJeTn0x6IYScVVyNc 	 
    	  
    		   
  mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
  mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     

           cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
   cornerSubPix mNf6TygbJ_3J7fN2nMZie 	 bwimage, Corners, cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
     Size mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   halfwsize,halfwsize mCJaKJz4Ks7wmAPeQv_Kr 	 
 , cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
     
 
  	 Size mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
  -1, -1 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
    ,cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    TermCriteria mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     cv mO4z_qd5n6igv9xR37S_c 	 
   TermCriteria mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
 MAX_ITER | cv mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
     
 
 TermCriteria mozCZQGgrEIkSImmzyXCE 	 
    	  
    EPS, 12, 0.005 mCJaKJz4Ks7wmAPeQv_Kr 	 
 mChDQC4Ii_VKZ3zxxDzUf 	 
 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		  
           
            mLiQxt6MiTu7N2jXYWYTn 	 
    	  
    		   
     
     
 
  mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		 unsigned  mcyd2SKTOMs686Zg6AvKk 	 
    	  
   i  mFkC0NYvfK1C9IpbEU8yT 	 
    0 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
 i  mDeJWrW4uoTHzHyLcNAet 	 
    	  
    		   
     
     
 
  	  DetectedMarkers.size mxnHnBAlV1zBx0pFgG2hE 	 
    	  
    		   
   mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
     
 i mJi3HfZE4vTdV03aCAimj 	 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    
                mZWWgl3f4HYSI9eS8vy0R 	 
    	  
    		   
     
     
  mNf6TygbJ_3J7fN2nMZie 	 
   int c  mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
  0 mQrv7hkcW3oFeWut6PStG 	 
    	  c  mmzqEYRn_LyctcsEDa2HQ 	 4 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
     
 
  c mz2NCdYNMJ4JtRGq6nF7t 	 muv3VsFUNwyEM5rzMcpVL 	 
    	  
 DetectedMarkers mR_dCFDAXIVEJ6jYEyn75 	 
    	  
    		   
i mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		   
     
     
 mNBurNSpxc9gKp7k6b2UJ 	 
    	  
    		   
     
     
 
  	c muL9BCFoOI9D7NibN8775 	 
    	  
    		   
     
     
 
  	   mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
    Corners mJYV46j2sr79QuUDayZBM 	 
    	  
   i * 4 + c mWtXsfGsUeXG7CDr4ZhYa 	 
    	  
    		   
     
 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
 
        mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
     
  
        
        mubMIpVJONhlCPblepzGH 	 
    	  
    		   
     
     
 
  DetectedMarkers mSr7ORshDX3Z3bwkW0_v4 	 
    	  
 msQ7kkP7s11kpylEAORwV 	 
    	  
    		   
     
 

 mcyd2SKTOMs686Zg6AvKk 	 
    	  
  MarkerDetector muH7u24saH7RlheXQke5v 	 
perimeter mkbagMvD3XTVPngjC8ADt 	 
    	  
const std muH7u24saH7RlheXQke5v 	 
    	  
    vector m_HS4b2QlfK4_jOJA7Lea 	 
    	  
   cv mTyitF2Lvx8UxPyne5zVe 	Point2f mXqTGLtmD8KolJCcRbqms 	 
    	  
    		   
  & a muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
  
 mjdc8VjjXr0Aa4BhmKceu 	 
    	  
    		 
     mcyd2SKTOMs686Zg6AvKk 	 
    	  
    		   
     sum  mFkC0NYvfK1C9IpbEU8yT 	 
    	  
    		   
     0 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     
 
 
     mLiQxt6MiTu7N2jXYWYTn 	 
    	  
    		   
     
     
 
 mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
     
 
  	size_t i  mGwWiiWvDznq22OfLuGn8 	 
    	  
    		   0 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   i  m_HS4b2QlfK4_jOJA7Lea 	 
    	  
     a.size mxnHnBAlV1zBx0pFgG2hE 	 
    	  
    		   
     
     
 
  	 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    	 i mX7kdfc21zA3ARvwWwPLE 	 
    	  
    		    muv3VsFUNwyEM5rzMcpVL 	
        sum mtCVzgEUPglVTOsfF7dtu 	 
   cv mTyitF2Lvx8UxPyne5zVe 	 
   norm mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
    a mJYV46j2sr79QuUDayZBM 	 
  i muL9BCFoOI9D7NibN8775 	 
    	  
 -a mCB5svCUyhmfwJcyEQuO0 	 
    	  mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
     
 
 i + 1 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		    % a.size mIG75xP7ccGaNq8etgGb4 	 
    	  
    		   
  mWtXsfGsUeXG7CDr4ZhYa 	 
    	  
    		   
  mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
    mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		 
     mBFaa1CPzTfO2iIvZnisG 	sum mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
 mnTDVIzEbmgX_VdpAhyDX 	 
    	  
    		   
 

 mDiKSPwqD6lHlVsVhQy0Q 	 
    	  
    		   
     
MarkerDetector mozCZQGgrEIkSImmzyXCE 	  getMarkerId mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   const cv mTyitF2Lvx8UxPyne5zVe 	 
    	  Mat &bits,  mrs3vsK0WttdjkZjbMoBv 	 
    	  
    		   
     &nrotations, const std mozCZQGgrEIkSImmzyXCE 	vector m_HS4b2QlfK4_jOJA7Lea 	 
    	  
    		uint64_t mAO3xiE22rc4cH6_sf2IU 	 
    	  
    		   
     
  &dict mZS9ceGnq7lgqx621Pk0t 	 
    	  
  muc7wUFAark9oWku8a18J 	
    
     mcmsoUXePN4OvUYc7IZ5Y 	 
    	  
    		   
     
     
 
  	  touulong mFkC0NYvfK1C9IpbEU8yT 	 
    	  
    		  mJYV46j2sr79QuUDayZBM 	 
    	 mjraYQwfX3l1LI3oi0oPf 	 
    	  
    		   
      mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
   const cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
Mat& code mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     
 
 
     mENOj2A1fouqj0opnjKoT 	 
    	
        std mTyitF2Lvx8UxPyne5zVe 	 
bitset mvY_F7NMDRsxqUYYdXTuC 	 
    	  
64 mXqTGLtmD8KolJCcRbqms 	 
    	  
    		   
      bits mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		  
         mcyd2SKTOMs686Zg6AvKk 	 
    	  
    		   
   bidx  mOpXAQmdZj7NQSNaN555V 	 
    	  
    	 0 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
         mlmXVChP8ipLLvIBiPpMW 	 
    	  
    		   
     
   mkbagMvD3XTVPngjC8ADt 	 
   int y  mOpXAQmdZj7NQSNaN555V 	 
    	  
    		    code.rows - 1 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  y  mrBBhc8z_AaoJbohTIyYv 	 
    	  
    		   
     0 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
      y mEWl1MspayioTL6dR2i2W 	 
  mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    	
             mlmXVChP8ipLLvIBiPpMW 	  mV9d5u9yWvHklE2mU8F7a 	 
    	  
   int x  mdvqaVuR9L5EaNhm_CXTs 	 
    	 code.cols - 1 mQrv7hkcW3oFeWut6PStG 	 x  mbiezpZGcIbw1fySzIM1b 	 
    	  
    		   
     0 mQrv7hkcW3oFeWut6PStG 	 
    	  
    	 x mJKEEwhFekmmUle8ts4Tl 	 
    	  
    		   
     
 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
 
                bits mCB5svCUyhmfwJcyEQuO0 	 
bidx mX7kdfc21zA3ARvwWwPLE 	 
    	  
 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		   
     
     
 
  mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
 code.at mvY_F7NMDRsxqUYYdXTuC 	 
   uchar mAO3xiE22rc4cH6_sf2IU 	 
    	   mV9d5u9yWvHklE2mU8F7a 	 
  y, x mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
      mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		 
         mBFaa1CPzTfO2iIvZnisG 	 
    	  
    		   
     
   bits.to_ullong mIG75xP7ccGaNq8etgGb4 	 
    	  
    		   
    mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     
     mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
  mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
  
     mJ9LikS_IYYtYLDVTRLpZ 	 
    	  
    		   
     rotate mOpXAQmdZj7NQSNaN555V 	 
    	  
    		   
 mJYV46j2sr79QuUDayZBM 	 
    	  
    		  mjraYQwfX3l1LI3oi0oPf 	 
    	  
    		   
     
     
 
  mkbagMvD3XTVPngjC8ADt 	 
    	  
    		const cv mO4z_qd5n6igv9xR37S_c 	 
Mat& in mZS9ceGnq7lgqx621Pk0t 	 
    	 
     me0iqslNZKxFAPWu7F5dZ 	 
    	  
    		   
     
     
 
 
        cv muH7u24saH7RlheXQke5v 	 
    	  Mat out mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
     
 
 in.size mIG75xP7ccGaNq8etgGb4 	 
    	  
    		,in.type mxnHnBAlV1zBx0pFgG2hE 	 
    	  
    		   
     
  muv3VsFUNwyEM5rzMcpVL 	 
    	  
    mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     

         mLiQxt6MiTu7N2jXYWYTn 	 
    	  
    		   
     
    mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
     
 
  	int i  mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
 0 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
 i  mvY_F7NMDRsxqUYYdXTuC 	 
    	  
     in.rows mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
 i mJi3HfZE4vTdV03aCAimj 	 
    	  
    		   
    mZS9ceGnq7lgqx621Pk0t 	 
    	  

             mLiQxt6MiTu7N2jXYWYTn 	 
    	  
    		   
     mlsICKRST_jneqGaXtuVv 	 
    	  
  int j  mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
     
 
   0 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
      j  m_HS4b2QlfK4_jOJA7Lea 	 
  in.cols mQrv7hkcW3oFeWut6PStG 	 
 j mz2NCdYNMJ4JtRGq6nF7t 	 
    	  
    		   
 muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
     
                out.at mmzqEYRn_LyctcsEDa2HQ 	 
 uchar mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
     
      mkbagMvD3XTVPngjC8ADt 	 
    	i, j mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
     
 
   mGwWiiWvDznq22OfLuGn8 	 
    	  
   in.at mDeJWrW4uoTHzHyLcNAet 	 
    	  
    		   
     
     
 
  	 uchar mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
 mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   in.cols - j - 1, i mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
      mrP_c8Ub6Qe1zacspWrJD 	 
    	
         mBFaa1CPzTfO2iIvZnisG 	 
    out mrP_c8Ub6Qe1zacspWrJD 	 
    	
     msQ7kkP7s11kpylEAORwV 	 
    	  mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     
 
 

     
     mqfhzPJTgCpv3KYlK5_7p 	 
    	  
    		   int x mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
     
 
0 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
x mvY_F7NMDRsxqUYYdXTuC 	 
    	  
    		   
     
     
 
  	 bits.cols mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
x ma3fDJqaln3qhdOZsRMM9 	 
   mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		    mENOj2A1fouqj0opnjKoT 	 
    	  
    	
         mVEphrDfp46VtXyNTATx7 	 
    	  
    		   
     
     
 
  bits.at mvY_F7NMDRsxqUYYdXTuC 	 
    	  
  uchar mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
     
     
 
  mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     0,x mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
     
  mGjZb54aQoLr5MfrERqM5 	 
  0 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    return -1 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    	
         mwnSSWvD5VxtXLdMAA854 	 
    	  
    		   
     
     
 bits.at m_HS4b2QlfK4_jOJA7Lea 	 
    	  
    		   
     
uchar mAO3xiE22rc4cH6_sf2IU 	 
 mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
     bits.rows-1,x mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
  mGjZb54aQoLr5MfrERqM5 	 
    	  
    		   
     
     
 
0 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
    return -1 mrP_c8Ub6Qe1zacspWrJD 	 

         mVEphrDfp46VtXyNTATx7 	 
    	  
    		   
     
     
 
  	 bits.at mmzqEYRn_LyctcsEDa2HQ 	 
   uchar mxTCLR2zVLUo76oVRUsFY 	 
    	  
    		   mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
     
 
  	 x,0 mChDQC4Ii_VKZ3zxxDzUf 	 
   mc4LhKWumx_776hOW_Kjl 	 
    	  
    		   
    0 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		return -1 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
    
         mVEphrDfp46VtXyNTATx7 	 
    	  
    		   
     
      bits.at mmzqEYRn_LyctcsEDa2HQ 	 
    	  
    		   uchar mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
     mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
     
 x,bits.cols-1 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
   mjEQi6f7v_DXF2H5tfOVg 	 
    	  
    		   
     
 0 mZS9ceGnq7lgqx621Pk0t 	return -1 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
     
 
 
     mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
     
 
     
    cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     
 Mat bit_inner mkbagMvD3XTVPngjC8ADt 	 
    	  
    	bits.cols-2,bits.rows-2,CV_8UC1 mZS9ceGnq7lgqx621Pk0t 	 
   mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		
     mqfhzPJTgCpv3KYlK5_7p 	 int r mOpXAQmdZj7NQSNaN555V 	 
    	  
    		   0 mSr7ORshDX3Z3bwkW0_v4 	r mvY_F7NMDRsxqUYYdXTuC 	 
    	  
bit_inner.rows mSr7ORshDX3Z3bwkW0_v4 	 
   r mz2NCdYNMJ4JtRGq6nF7t 	 
    	  
     mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
     
 
  	 
         mtMYa5oQ0r8zESiyoRoz5 	 
    	  
    	int c mOpXAQmdZj7NQSNaN555V 	 
    	  0 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    	c mvY_F7NMDRsxqUYYdXTuC 	 
    	  
    		   
     
     
 
  	 bit_inner.cols mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
  c mz2NCdYNMJ4JtRGq6nF7t 	 
    	 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  

            bit_inner.at mvY_F7NMDRsxqUYYdXTuC 	 
    	  
    		   
     
uchar mAO3xiE22rc4cH6_sf2IU 	 
    	  
    		   
     
  mlsICKRST_jneqGaXtuVv 	 
    	  
  r,c mChDQC4Ii_VKZ3zxxDzUf 	 
  mGwWiiWvDznq22OfLuGn8 	 
    	  
    		   
     
 bits.at mmzqEYRn_LyctcsEDa2HQ 	 
    	  
    		   
uchar mXqTGLtmD8KolJCcRbqms 	 mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
    r+1,c+1 muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		    mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    	

    
    nrotations  mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		 0 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
   
    do
     me0iqslNZKxFAPWu7F5dZ 	 
    	  
    		 
         mJ9LikS_IYYtYLDVTRLpZ 	 
    	  
    		   
     id mFkC0NYvfK1C9IpbEU8yT 	 
    	  
   touulong mkbagMvD3XTVPngjC8ADt 	 
    	  
    		 bit_inner muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
     
  mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
  
        
         mENIjjNjrAgZCfwdaAviw 	 
    	  
size_t i mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		  0 mSr7ORshDX3Z3bwkW0_v4 	i mDeJWrW4uoTHzHyLcNAet 	 
    	  
    	dict.size mbx3fJeTn0x6IYScVVyNc 	 
    	  
    		   
     
     mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
   i mz2NCdYNMJ4JtRGq6nF7t 	 
     mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
 
  	 
             mVEphrDfp46VtXyNTATx7 	 
    	  
    dict mJYV46j2sr79QuUDayZBM 	 
    	  
 i muL9BCFoOI9D7NibN8775 	 
    	   mddU3cJATXGE9cIrKKmHG 	 
    	  
id mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     

                 mubMIpVJONhlCPblepzGH 	 
i mrP_c8Ub6Qe1zacspWrJD 	 
    	  

        bit_inner  mOpXAQmdZj7NQSNaN555V 	 
    	  
   rotate mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
     
 
  bit_inner mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     
 
  mSr7ORshDX3Z3bwkW0_v4 	 
        nrotations mX7kdfc21zA3ARvwWwPLE 	 
    	  
    		   
     
     
 
  	 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		
     mkUPTJY8FnecYLodH0z5w 	 
    while  mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
 nrotations  mDeJWrW4uoTHzHyLcNAet 	 
    	  
    		   
  4 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
 
  mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     

     mBFaa1CPzTfO2iIvZnisG 	 
    	  
    		   
  -1 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
   
 msQ7kkP7s11kpylEAORwV 	 
    	

 mPJiY4IaTWfF3csqf0juJ 	 
  MarkerDetector mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
     
 getSubpixelValue mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
     
 
  const cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     Mat &im_grey,const cv muH7u24saH7RlheXQke5v 	 
    	  
 Point2f &p mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
 muc7wUFAark9oWku8a18J 	 
    	  
    	

      mPJiY4IaTWfF3csqf0juJ 	 
    	  
    		   
     intpartX mQrv7hkcW3oFeWut6PStG 	 
    	 
     mPJiY4IaTWfF3csqf0juJ 	 
    	  
    		   
     
 decpartX mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
   std muH7u24saH7RlheXQke5v 	 
    	  
    		   
   modf mlsICKRST_jneqGaXtuVv 	 
 p.x,&intpartX muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		  
     mkCOs11tlwA6W0mJF14_x 	 
    	  
    		   
     
 intpartY mSr7ORshDX3Z3bwkW0_v4 	 

     mkCOs11tlwA6W0mJF14_x 	 
    	  
    		decpartY mFkC0NYvfK1C9IpbEU8yT 	 
std mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     
 
  	modf mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
p.y,&intpartY mZS9ceGnq7lgqx621Pk0t 	 
    	  
   mTmr0dyQ5pfW9Bxa5rfQf 	 
    	 

    cv mozCZQGgrEIkSImmzyXCE 	Point tl mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     
 
 

     mtuMQpxdfoxJkmf_Rfq0d 	  mkbagMvD3XTVPngjC8ADt 	 
    	  
    decpartX mXqTGLtmD8KolJCcRbqms 	 
   0.5 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
       me0iqslNZKxFAPWu7F5dZ 	 

         mB6HP6k2xlgUz2wqXczwe 	 
    	  
    		   
 mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
     
 
  	decpartY mxTCLR2zVLUo76oVRUsFY 	 
    	  
  0.5 mChDQC4Ii_VKZ3zxxDzUf 	  tl mFkC0NYvfK1C9IpbEU8yT 	 
    	  
  cv muH7u24saH7RlheXQke5v 	 
    Point mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     intpartX,intpartY mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     
 
   mQrv7hkcW3oFeWut6PStG 	 
 
         mj8FJJadUjRZ1AZ2HpuHs 	tl mGwWiiWvDznq22OfLuGn8 	 
    	cv mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
     
 
 Point mV9d5u9yWvHklE2mU8F7a 	 
   intpartX,intpartY-1 mCJaKJz4Ks7wmAPeQv_Kr 	 
   mrP_c8Ub6Qe1zacspWrJD 	 
    	  

     mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
     
    
    else me0iqslNZKxFAPWu7F5dZ 	
         mB6HP6k2xlgUz2wqXczwe 	 mlsICKRST_jneqGaXtuVv 	 
   decpartY mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
     
     
 
 0.5 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
     
 
  tl mdvqaVuR9L5EaNhm_CXTs 	 
    	  cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
     
     
 
  	 Point mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
     
 
 intpartX-1,intpartY mZS9ceGnq7lgqx621Pk0t 	 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
     

         mj8FJJadUjRZ1AZ2HpuHs 	 
    	  
    		   
     
     tl mFkC0NYvfK1C9IpbEU8yT 	 
    	  
    		   
     
     cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		 Point mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
     
 
  	 intpartX-1,intpartY-1 muv3VsFUNwyEM5rzMcpVL 	 
 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
   
     mc0o_c5zaAv5kGOLHP9dv 	 
  
     mwnSSWvD5VxtXLdMAA854 	 
    	  
    		   
tl.x mmzqEYRn_LyctcsEDa2HQ 	 
    	  
    		   
    0 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		    tl.x mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
     0 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
     mGkHoKSh0MRSiy3mkSlZb 	 
    	  
    		   
     
     
 
tl.y mmzqEYRn_LyctcsEDa2HQ 	 
    	  
 0 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
 tl.y mFkC0NYvfK1C9IpbEU8yT 	 
0 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
     
 
 
     mV1QloqXovtxfBjycQ__r 	 
    	  
    		   
     
tl.x m_Ah6NKdNgYPZEM8vImlM 	 
    	  
    		   
     
    im_grey.cols mCJaKJz4Ks7wmAPeQv_Kr 	 
   tl.x mdvqaVuR9L5EaNhm_CXTs 	 
    	  im_grey.cols-1 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
     mVEphrDfp46VtXyNTATx7 	 
    	  
    		  tl.y mG2qGkHXgoSG0kBsCveUo 	 
    	  
 im_grey.cols mCJaKJz4Ks7wmAPeQv_Kr 	tl.y mGwWiiWvDznq22OfLuGn8 	 
 im_grey.rows-1 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   

     mfokMjHARARzwmiqx8DMa 	 
    	  
   mNf6TygbJ_3J7fN2nMZie 	 
    1.f-decpartY mZS9ceGnq7lgqx621Pk0t 	 
  * mlsICKRST_jneqGaXtuVv 	 
  1.-decpartX muv3VsFUNwyEM5rzMcpVL 	 
   *float mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
im_grey.at mmzqEYRn_LyctcsEDa2HQ 	 
    	  
    		   
     
    uchar mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
     
      mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
  tl.y,tl.x muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
    muv3VsFUNwyEM5rzMcpVL 	 
    	  
    +
            decpartX* mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
     
 1-decpartY mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    	*float mlsICKRST_jneqGaXtuVv 	 
    	  
   im_grey.at mmzqEYRn_LyctcsEDa2HQ 	 
    	  
    		   
     
     
 
  uchar mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
      mV9d5u9yWvHklE2mU8F7a 	 
    	  
    tl.y,tl.x+1 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
     +
             mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
 1-decpartX mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    *decpartY*float mV9d5u9yWvHklE2mU8F7a 	 
    	im_grey.at m_HS4b2QlfK4_jOJA7Lea 	 
    	  
    		   
     
     
 
  uchar mxTCLR2zVLUo76oVRUsFY 	 
    	  
    		   
  mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
     
 
 tl.y+1,tl.x mChDQC4Ii_VKZ3zxxDzUf 	 
    	 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
 
  	 +
            decpartX*decpartY*float mlsICKRST_jneqGaXtuVv 	 
    	im_grey.at m_HS4b2QlfK4_jOJA7Lea 	uchar mXqTGLtmD8KolJCcRbqms 	 
    	  
    		   
      mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
 tl.y+1,tl.x+1 muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
    mChDQC4Ii_VKZ3zxxDzUf 	 mrP_c8Ub6Qe1zacspWrJD 	 
    	 
 msQ7kkP7s11kpylEAORwV 	 
    	


Marker  MarkerDetector mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
sort mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
 const  Marker &marker mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     
 
  	  muc7wUFAark9oWku8a18J 	 
    	  
    		   
     
     
 
  	 
    Marker res_marker mGwWiiWvDznq22OfLuGn8 	 marker mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
 
    
        
        
         mFRzpc9yiC1RRp9H3n66u 	 
    	  
   dx1  mOpXAQmdZj7NQSNaN555V 	 
    	  
    		  res_marker mCB5svCUyhmfwJcyEQuO0 	 
    	  1 mWtXsfGsUeXG7CDr4ZhYa 	 
    .x - res_marker mR_dCFDAXIVEJ6jYEyn75 	 
    	  
    		   
    0 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		   
     
     
 
 .x mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   
     
         mbFRqKDhxtmARxx6OFpRB 	 
    	  
    		dy1  mFkC0NYvfK1C9IpbEU8yT 	 
    	  res_marker mNBurNSpxc9gKp7k6b2UJ 	 
    	  
    		   
     
     
1 mWtXsfGsUeXG7CDr4ZhYa 	 .y - res_marker mCB5svCUyhmfwJcyEQuO0 	 
    	0 muL9BCFoOI9D7NibN8775 	 
    	  
 .y mTmr0dyQ5pfW9Bxa5rfQf 	 

         mbFRqKDhxtmARxx6OFpRB 	 
    	  
    		   
     dx2  mOpXAQmdZj7NQSNaN555V 	 
    	  
    		   
     
   res_marker mCB5svCUyhmfwJcyEQuO0 	 
    	  
    		   
     
 2 mjraYQwfX3l1LI3oi0oPf 	 
    	  
    		   
     
 .x - res_marker mJYV46j2sr79QuUDayZBM 	 
    	  
    		   
     0 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		   
     
     
 
  .x mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
 
         mbFRqKDhxtmARxx6OFpRB 	 
    	  
    dy2  mFkC0NYvfK1C9IpbEU8yT 	 
 res_marker mJYV46j2sr79QuUDayZBM 	 
    	 2 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		  .y - res_marker mR_dCFDAXIVEJ6jYEyn75 	 
0 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		   
     
     
 
 .y mSr7ORshDX3Z3bwkW0_v4 	 
    	
         mFRzpc9yiC1RRp9H3n66u 	 
    	  
o  mOpXAQmdZj7NQSNaN555V 	 
    mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
 dx1 * dy2 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    	 -  mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
  dy1 * dx2 muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
    mrP_c8Ub6Qe1zacspWrJD 	 

         mB6HP6k2xlgUz2wqXczwe 	 
    	  
  mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
   o  mmzqEYRn_LyctcsEDa2HQ 	 
    	  
  0.0 mChDQC4Ii_VKZ3zxxDzUf 	 
         me0iqslNZKxFAPWu7F5dZ 	 
    	  
    		   
     
     
   
            std mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   swap mV9d5u9yWvHklE2mU8F7a 	 
    	  res_marker mR_dCFDAXIVEJ6jYEyn75 	 
1 muL9BCFoOI9D7NibN8775 	 
    	  
    		   , res_marker mCB5svCUyhmfwJcyEQuO0 	 
    	  
    		   
   3 muL9BCFoOI9D7NibN8775 	 
    	  
    		   
 muv3VsFUNwyEM5rzMcpVL 	 
    	 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
 
         msQ7kkP7s11kpylEAORwV 	 
    	  
    		   
     
  
     mtAVV0SeDoEyXWqGXyJEt 	 
    	res_marker mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
 
 mkUPTJY8FnecYLodH0z5w 	 
    	  
    		   
     
     
 
  	 


 mE_CbfJmYdWI4XC4EBTpM 	Marker muH7u24saH7RlheXQke5v 	 
    	  
    		   
     
  draw mV9d5u9yWvHklE2mU8F7a 	cv mO4z_qd5n6igv9xR37S_c 	 
 Mat &in, const cv muH7u24saH7RlheXQke5v 	 
    	  
Scalar color mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
 const mENOj2A1fouqj0opnjKoT 	 
    	  
    		   
     
     
 
  
     mcmsoUXePN4OvUYc7IZ5Y 	 
    	  
    		   
     
     
 
  _to_string mFkC0NYvfK1C9IpbEU8yT 	 
    	  
    		   
  mCB5svCUyhmfwJcyEQuO0 	 
    	  
    		   
   mWtXsfGsUeXG7CDr4ZhYa 	 
    	  
    		   
     
     
 mV9d5u9yWvHklE2mU8F7a 	 
    	  
  int i muv3VsFUNwyEM5rzMcpVL 	 mjdc8VjjXr0Aa4BhmKceu 	 
    	  
    		 
        std mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
     
 
  stringstream str mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		str miaamf1J56VfFHp2UKyZI 	 
    	  
  i mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		 mBFaa1CPzTfO2iIvZnisG 	 
    	  
    str.str mxnHnBAlV1zBx0pFgG2hE 	 
   mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
     
 
  
     mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
  mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
  

     mkCOs11tlwA6W0mJF14_x 	 
    	  
    		   
     
     
 
 flineWidth mOpXAQmdZj7NQSNaN555V 	 
    	  
    		   
     
     
 
  	   std mTyitF2Lvx8UxPyne5zVe 	 
    	  
 max mV9d5u9yWvHklE2mU8F7a 	1.f, std mO4z_qd5n6igv9xR37S_c 	 min mNf6TygbJ_3J7fN2nMZie 	 
    	  5.f, float mNf6TygbJ_3J7fN2nMZie 	 
    	  
  in.cols muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
     
 
  / 500.f mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
  mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
    mQrv7hkcW3oFeWut6PStG 	
     mrs3vsK0WttdjkZjbMoBv 	 
    	  
    		   
     
     
 
lineWidth mFkC0NYvfK1C9IpbEU8yT 	 
    	   round mV9d5u9yWvHklE2mU8F7a 	 
    	  
 flineWidth mZS9ceGnq7lgqx621Pk0t 	 mQrv7hkcW3oFeWut6PStG 	 
    	  
    		   
     
     mENIjjNjrAgZCfwdaAviw 	 
int i mFkC0NYvfK1C9IpbEU8yT 	 
    	  
    		   
     
     
 
  	 0 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
  i mvY_F7NMDRsxqUYYdXTuC 	 
    	  
    		   
     
     
4 mSr7ORshDX3Z3bwkW0_v4 	 
    	  
    		   i mJi3HfZE4vTdV03aCAimj 	 
    	  
    		   
     
     
 
  	  mZS9ceGnq7lgqx621Pk0t 	 
    	  
 
        cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     
 
  line mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
   in,  mNf6TygbJ_3J7fN2nMZie 	 
    	  
 *this mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
      mJYV46j2sr79QuUDayZBM 	 
    	  
 i mjraYQwfX3l1LI3oi0oPf 	 
    	  
    		   
     
 ,  mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
    *this mZS9ceGnq7lgqx621Pk0t 	 
    	  
  mJYV46j2sr79QuUDayZBM 	 
    	  
    		   
     
 mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
    i+1  mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
     
     
 
  	%4 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		   
     
     , color, lineWidth mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
 mSr7ORshDX3Z3bwkW0_v4 	

     mJ9LikS_IYYtYLDVTRLpZ 	 
    	  
    		   
     
     
 
 p2  mdvqaVuR9L5EaNhm_CXTs 	 
    	  
    		   
     
  cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
     
 
Point2f mV9d5u9yWvHklE2mU8F7a 	 
    	  2.f * static_cast mDeJWrW4uoTHzHyLcNAet 	 
    	  
 float mEcbmE2N2oE_6qGHcyJ4M 	 
    	  
    		   
 mNf6TygbJ_3J7fN2nMZie 	 
    	  lineWidth mZS9ceGnq7lgqx621Pk0t 	 
    	  
    , 2.f * static_cast mDeJWrW4uoTHzHyLcNAet 	 
    	  
    		   
 float mxTCLR2zVLUo76oVRUsFY 	 
   mlsICKRST_jneqGaXtuVv 	 lineWidth mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
  mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   

    cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
    rectangle mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
     
 
  	 in,  mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
     *this mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
 
  mR_dCFDAXIVEJ6jYEyn75 	 
    	0 mcUA0TPPLmQ9fBms5eB4L 	 
    	  
    		    - p2,  mlsICKRST_jneqGaXtuVv 	 
    	  
    		  *this mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
  mR_dCFDAXIVEJ6jYEyn75 	 
    	  
    		   0 mjraYQwfX3l1LI3oi0oPf 	 
    	  
    		   
     
     
 
  	 + p2, cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		Scalar mlsICKRST_jneqGaXtuVv 	 
    	  
    		0, 0, 255, 255 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     , -1 mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
  mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
     
 
  	 
    cv mO4z_qd5n6igv9xR37S_c 	 rectangle mNf6TygbJ_3J7fN2nMZie 	 
    	  
in,  mV9d5u9yWvHklE2mU8F7a 	 
  *this mCJaKJz4Ks7wmAPeQv_Kr 	 mNBurNSpxc9gKp7k6b2UJ 	 
    	  
    		   
     1 mWtXsfGsUeXG7CDr4ZhYa 	 
    	  
    	 - p2,  mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     *this mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
   mNBurNSpxc9gKp7k6b2UJ 	 
    	  
    		  1 mjraYQwfX3l1LI3oi0oPf 	 
    	  
    		  + p2, cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    	Scalar mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
     
   0, 255, 0, 255 muv3VsFUNwyEM5rzMcpVL 	 
    , lineWidth mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   
     
     
 mQrv7hkcW3oFeWut6PStG 	 
   
    cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
     
 
  	 rectangle mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
     
 
  	 in,  mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     *this mCJaKJz4Ks7wmAPeQv_Kr 	 
    	  
    		   mNBurNSpxc9gKp7k6b2UJ 	 
 2 mjraYQwfX3l1LI3oi0oPf 	 
    	 - p2,  mNf6TygbJ_3J7fN2nMZie 	 
    	*this muv3VsFUNwyEM5rzMcpVL 	 
    	  
 mNBurNSpxc9gKp7k6b2UJ 	 
    	  
    		   
     
 2 mjraYQwfX3l1LI3oi0oPf 	 
    	  
    		   
     
      + p2, cv mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
    Scalar mV9d5u9yWvHklE2mU8F7a 	 
    	  
    		   
255, 0, 0, 255 muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
   , lineWidth muv3VsFUNwyEM5rzMcpVL 	 
    	   mTmr0dyQ5pfW9Bxa5rfQf 	 
    	

    
    cv mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     Point2f cent mkbagMvD3XTVPngjC8ADt 	 
    	  
    		   
     
     
 
  	 0, 0 mChDQC4Ii_VKZ3zxxDzUf 	  mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   

     mtMYa5oQ0r8zESiyoRoz5 	 
    	  
    	auto &p:*this muv3VsFUNwyEM5rzMcpVL 	 
    	  
    		   
     
     
 
  	  cent mQMjF2zNgqH5ntgOHOAT_ 	 
    	  
    		   
p mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
     
     
 
  	 
    cent mfGAX5elGMZVlX6yU1BWi 	 
    	  
    		 4 mTmr0dyQ5pfW9Bxa5rfQf 	 
    	  
    		   
     
   
     mkCOs11tlwA6W0mJF14_x 	 
    	  
    		   
     
     
 
  	fsize mdvqaVuR9L5EaNhm_CXTs 	  std mozCZQGgrEIkSImmzyXCE 	 
    	  
    		   
     
  min mlsICKRST_jneqGaXtuVv 	 
    	  
    		   
     
     
3.0f, flineWidth * 0.75f muv3VsFUNwyEM5rzMcpVL 	 mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		   
  
    cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
    putText mNf6TygbJ_3J7fN2nMZie 	in,_to_string mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
     
 
  	 id mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
    , cent-cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
 Point2f mNf6TygbJ_3J7fN2nMZie 	 
    	  
    		   
     
 10*flineWidth,0 mZS9ceGnq7lgqx621Pk0t 	 
    	  
    		   
     
   ,  cv mO4z_qd5n6igv9xR37S_c 	 
    	  
    		   
     
    FONT_HERSHEY_SIMPLEX,fsize,cv mTyitF2Lvx8UxPyne5zVe 	 
    	  
    		   
     
     
 
 Scalar mkbagMvD3XTVPngjC8ADt 	 
    	  
    255,255,255 mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		  -color, lineWidth,cv muH7u24saH7RlheXQke5v 	 
    	  
    		   
     
     
 LINE_AA mChDQC4Ii_VKZ3zxxDzUf 	 
    	  
    		   
   mrP_c8Ub6Qe1zacspWrJD 	 
    	  
    		  
 mkUPTJY8FnecYLodH0z5w 	 
   
 mc0o_c5zaAv5kGOLHP9dv 	 
    	  
    		   
     
 
#endif

#ifdef _3994214971530577637
#undef  mu1zMPxFJ6lolSo8kFpv4 
#undef  mAO3xiE22rc4cH6_sf2IU 
#undef mzYgRDSGqLXuOnzzctSLNm2eMa3K1Ur
#undef  mXqTGLtmD8KolJCcRbqms 
#undef mpj1OroLWAliM3cpojLaIGrVyypg_Ps
#undef  mH_k21Ay0zlgsDrkgqxsR 
#undef  mSIVwLRfGMC4DMDmuBHFR 
#undef  mDeJWrW4uoTHzHyLcNAet 
#undef  mbK0209bgL68cKiD82YsP 
#undef mR8qFMsLupcMJiW3whVMumDqncBlBld
#undef m_TM6hAgk6CVshBOL_5Mnadve75apGl
#undef  mR_dCFDAXIVEJ6jYEyn75 
#undef  mFRzpc9yiC1RRp9H3n66u 
#undef  muH7u24saH7RlheXQke5v 
#undef  mtWEEdxmpGSyLVMuZTvyp 
#undef  mhoPkn3UvU_xVsJBhPgzi 
#undef  mddU3cJATXGE9cIrKKmHG 
#undef  mE_CbfJmYdWI4XC4EBTpM 
#undef  mUNmJJEJUl0qPANbj_Eu7 
#undef  ma3fDJqaln3qhdOZsRMM9 
#undef  mZlZaG4WchOlShWfsH8tE 
#undef  mrE2EjR5qaFb4rGkGOBA2 
#undef  mtAVV0SeDoEyXWqGXyJEt 
#undef  mlmXVChP8ipLLvIBiPpMW 
#undef  mX7kdfc21zA3ARvwWwPLE 
#undef  mtCVzgEUPglVTOsfF7dtu 
#undef  mbiezpZGcIbw1fySzIM1b 
#undef  mbx3fJeTn0x6IYScVVyNc 
#undef  mqfhzPJTgCpv3KYlK5_7p 
#undef mYugpe5ySPHkiJRdaSR0xrGAInKGlPv
#undef  ml5Jd2lQZ6qMJyg0OFgvU 
#undef  mlsICKRST_jneqGaXtuVv 
#undef  mCB5svCUyhmfwJcyEQuO0 
#undef  miaamf1J56VfFHp2UKyZI 
#undef  mK6HLbeH5XHOvKuN4NsQ3 
#undef  mXGhWrKl84vifpLlc0PC6 
#undef  mpkQ_NixIVAxG411sopSo 
#undef mX1gwMiNsVrnsa3N4UB1kOe3c7vMhcQ
#undef mL1iFGnNT9pJkL9uSBIg0MBbimeIJ_1
#undef  mJ8SsCY3h0qTwGHxMtwHf 
#undef mu3mz3orWipWJ8YothsWsQTyqHH71TM
#undef  mL0NQ9ktzvPmme3M4fyCQ 
#undef  mg9Twy4YcltH0wr4RXS5A 
#undef  mNPrCB7OD4mubLerN3_4F 
#undef  mLnBceSdpOwDVPsTbMUMe 
#undef  mu8VMFkP7E4jr6Tdw4Um0 
#undef  mkCOs11tlwA6W0mJF14_x 
#undef  mWtXsfGsUeXG7CDr4ZhYa 
#undef  mkpkwFQMnjUlqIxoJg_Uf 
#undef  mAAmDwlmS9u0xxdN5g0af 
#undef  mfGAX5elGMZVlX6yU1BWi 
#undef  mV1QloqXovtxfBjycQ__r 
#undef  mut3SPyv8WHCtfn9e3Sai 
#undef  mrr0ibeCpvFbowBxHOisH 
#undef  mDP0JMOAHgqSQlFEviaLG 
#undef  mJ9LikS_IYYtYLDVTRLpZ 
#undef  mDiKSPwqD6lHlVsVhQy0Q 
#undef  mzuwBB__J5sv3IUQADE_3 
#undef  m__lcvPHn8NGsWyPIr3p5 
#undef  m_HS4b2QlfK4_jOJA7Lea 
#undef  mEWl1MspayioTL6dR2i2W 
#undef  mVEphrDfp46VtXyNTATx7 
#undef  mYsUozr4oIEHD4bddBjCO 
#undef  mfokMjHARARzwmiqx8DMa 
#undef  mldlYdrCajgJIXywENQCF 
#undef mhTsLKcKPl4G_GlSl1uhVPwyZ4O0hL_
#undef  mcMgwB0EzJV1Gsszem6QV 
#undef  mCbMSEzewB2ZcuX5X3Lr2 
#undef  mENOj2A1fouqj0opnjKoT 
#undef  msiY4PnEbrOoM_LaYIfkH 
#undef  mdvqaVuR9L5EaNhm_CXTs 
#undef maRVsYW0j9L50OAmNHeEmVq1N2HwNI4
#undef  mBFaa1CPzTfO2iIvZnisG 
#undef  mtuMQpxdfoxJkmf_Rfq0d 
#undef  mkUPTJY8FnecYLodH0z5w 
#undef  mrpmhTgnwQmY4FcclKohM 
#undef mlPTUbFx7KPjY6O19pf3CxNMZm_NiGE
#undef mNUou5IeA0397bgONqyXfQywlHkqvXF
#undef  mAncUSyQDaEPJSOcL99sq 
#undef  mTyitF2Lvx8UxPyne5zVe 
#undef mX1nAiQj1qqu1vMMc_kqLJbtPm03XO0
#undef  msQ7kkP7s11kpylEAORwV 
#undef  mJYV46j2sr79QuUDayZBM 
#undef  mz0vd7EthNjfO8_IyhuwM 
#undef mSHUdm8RFONaz6OG7KGgkqjNEg3MKNf
#undef  mkbagMvD3XTVPngjC8ADt 
#undef  mbhh6cFhSlDR38p8LHY7q 
#undef  mO4z_qd5n6igv9xR37S_c 
#undef  mQrv7hkcW3oFeWut6PStG 
#undef  mHUJysfeF2Zbv3qqxvwd7 
#undef mP9AqOgEhSf2BgXFs1KT1xuWo_NtsV5
#undef  mjElzh5LfqP7l_DSgQXJ8 
#undef  mjSzXSN9XZVACffhnj8kA 
#undef  mgYBhehm0Yp7agVfMymco 
#undef  mXzUaqhTSAFMf6iqaeeiJ 
#undef  mNBurNSpxc9gKp7k6b2UJ 
#undef  mQMjF2zNgqH5ntgOHOAT_ 
#undef myWZyeSG59NQTc7WOAGLrrC_iZyvSai
#undef  m_9H9gofCHSCZF99FnPdn 
#undef  mLiQxt6MiTu7N2jXYWYTn 
#undef mH7OCR57Nc36d3a66hkRwgTYkD2qoT9
#undef  mjEQi6f7v_DXF2H5tfOVg 
#undef  mC01hioRqWayaWmBKLCAq 
#undef  mjraYQwfX3l1LI3oi0oPf 
#undef  mj8FJJadUjRZ1AZ2HpuHs 
#undef mPtOnOuSPPDJUtXxRCiBDOvj_IJ01QI
#undef  mEcbmE2N2oE_6qGHcyJ4M 
#undef  mdPVeeRLDncG9ZENPgYpa 
#undef  mENIjjNjrAgZCfwdaAviw 
#undef  myKtCQyK_KHpnIiaFOUf0 
#undef  mJKEEwhFekmmUle8ts4Tl 
#undef  mkNk2EG1uYSLZ6KwBs0gC 
#undef  mlBR2FIiemMophFyr5rD7 
#undef  m_DLJbOzXcNRU0fxu99mU 
#undef  mTSmn4cA4cZLuH9hEBC3B 
#undef  mzN5F8McKXe12eIdP5_Ah 
#undef  mcmsoUXePN4OvUYc7IZ5Y 
#undef  mrP_c8Ub6Qe1zacspWrJD 
#undef  mzHWTrFT67ZmR11dwYYTh 
#undef  muc7wUFAark9oWku8a18J 
#undef  msh6wVvNu3UUYaSoHD1xM 
#undef  mChDQC4Ii_VKZ3zxxDzUf 
#undef  mr5RFEb3D7r7vO9qqRO88 
#undef  mq5oi1iqBSJjGA1VZs3SA 
#undef  mpHDccSehSaPSnQzG5eYu 
#undef  mUTSZ__S1SdK2KG45DWUm 
#undef  mqNI4_fE9V47075t2juoY 
#undef  mz2NCdYNMJ4JtRGq6nF7t 
#undef mFRuLsRjOlvH9YoJcWfADtzvMa1EXwp
#undef  mrm9MMQlr5kBYHIvZBqtL 
#undef  mmzqEYRn_LyctcsEDa2HQ 
#undef mHgMY2zK4RtFDjwrHcFHVSpFc2d5yOm
#undef mZ5ixWcfKHJaejg72nqTTaem5XCBf9H
#undef mSOTXnOIL0xf8ITnQqPYQYJioYSp8Z1
#undef  mHQ8TCR7AwoYSg71Swyon 
#undef  mIG75xP7ccGaNq8etgGb4 
#undef  mTmr0dyQ5pfW9Bxa5rfQf 
#undef  mOpXAQmdZj7NQSNaN555V 
#undef  msXI9xNqJ2saBfHANEmYG 
#undef  mB6HP6k2xlgUz2wqXczwe 
#undef  myrEt7Aa_7wBrbOTCoeEC 
#undef  mjCHnKVbk8OwmtJRy_SMR 
#undef  muhDy5OBINDfgZ84otbf4 
#undef  mESY4bzIrBZttLcxRnXLR 
#undef  mG658aG9Od_wV7xESL9m1 
#undef  mKygmBqwvycF2nOg7aeDW 
#undef monPHH1dnULVla7ElbyonFJvO11Jd1a
#undef  mMmUtHiw3cbE8z5afEJkB 
#undef  mVuw3F_iEYLm1xQ766ZMz 
#undef  mwueBuJPDOTh5DbXMUAy4 
#undef  m_Ah6NKdNgYPZEM8vImlM 
#undef  maaDMWVjEN1BPCsrI_ByK 
#undef  mWE_Xtu5PLl3ZxcqzGQHn 
#undef  mJi3HfZE4vTdV03aCAimj 
#undef  mubMIpVJONhlCPblepzGH 
#undef mSLGPvqVw9ieY_7Gn92EQG2sC8gBUb7
#undef  muv3VsFUNwyEM5rzMcpVL 
#undef  mcKeF8As76VMY3D1Qb4AL 
#undef  me0iqslNZKxFAPWu7F5dZ 
#undef  mfOUQBriVsfBzwdnMRAws 
#undef  mPomwpZH1eAiR4L2yjxwY 
#undef  mGwWiiWvDznq22OfLuGn8 
#undef mTef7hytVjFm9VUpxBhWpuPIqB3HyUT
#undef  malSnpXEOSRzWNB_pNE5v 
#undef  muL9BCFoOI9D7NibN8775 
#undef  mwnSSWvD5VxtXLdMAA854 
#undef  mtbDKUmoNkdILdWy9sWry 
#undef  mZkrSYojz5UMn5BPLfPNx 
#undef mgra4bGx8yCZ1HEOa4sarcEM6TZZbf_
#undef mLUdfVYu3VQSVRtc4GZoF173v0S92bq
#undef  mc0o_c5zaAv5kGOLHP9dv 
#undef mIdl54FwZclwXRU8RT_pK0X0Buoetqz
#undef  mzEUQfYnaMzIgGaHKRhwp 
#undef  mNBT9hkF6X8AHBMc38rxG 
#undef mqegKF_0kZofyyRQmCqJOJzr8hFWuj9
#undef  mGjZb54aQoLr5MfrERqM5 
#undef  mJS53DyDTBfeVCSmbTHvN 
#undef  mPZL1ZDoOfdToCS6G2uVf 
#undef  mOapFcKZasqqnDpwehsj7 
#undef  mozCZQGgrEIkSImmzyXCE 
#undef  mrs3vsK0WttdjkZjbMoBv 
#undef  mb0l2UNOzi2myV4UMN_l5 
#undef  mnTDVIzEbmgX_VdpAhyDX 
#undef mkRdTH9VEjGb65T_qmET9nqprjGcqBc
#undef  mn6AEuWg2QYne4JR6fLg1 
#undef  mcUA0TPPLmQ9fBms5eB4L 
#undef  mrabaTcHMOpAvxVb7IvbA 
#undef  myoLEtYQKNlKuCVDB9k7g 
#undef mz6OyKObXJsCO4AEGAzcpAd61zBV5kG
#undef  mYdmvEo23jHfrA6mWubnL 
#undef  mjFBQiPEMePvg1MABqzYm 
#undef  myhw_rdPVb6jTTA9isti7 
#undef  mKAanxWKURsCl8ozhCL65 
#undef  mQ3_kYmccnIropzQ3MxoC 
#undef  mUoaRimKhQVjiR5fHxxmF 
#undef mY1_eb3cBfHo327LPh23E9HUoVnIw26
#undef  mJ00zryslayel_JxYZrVW 
#undef  mFkC0NYvfK1C9IpbEU8yT 
#undef  mGzQZhixtGZgkfCG7eFjp 
#undef  mRH69tESmocqV1IOfpgew 
#undef  mV9d5u9yWvHklE2mU8F7a 
#undef  mPE8Fcn2GRzlYYaiJPBab 
#undef  mNlhcH6MwCHjbJvpthYqK 
#undef  mzzYhbznZ7So0wJl0ZYQ7 
#undef  mNf6TygbJ_3J7fN2nMZie 
#undef  mGkHoKSh0MRSiy3mkSlZb 
#undef mbDPGvJ8SKfQTWEGdMbhA7NJx7idF6f
#undef  mbxKd60T0jbBIj0UjXlRF 
#undef  mPJiY4IaTWfF3csqf0juJ 
#undef  mSr7ORshDX3Z3bwkW0_v4 
#undef  msaKAlxEDXv__6GBtcFRO 
#undef mzncRYC7sxYHHT1Lrp1UqW7lPaBjzzv
#undef  mIxFTBL7R5xnN9NUOyTu1 
#undef  mbFRqKDhxtmARxx6OFpRB 
#undef  mvm3NyvyBKLdqNKpTDCkp 
#undef mVHVidfIGRoNh4dGUV7Qktl5Ayc9JTb
#undef mJRDhq6_5RbuQDpJ5SRyAFVB5_Qd6FC
#undef  mCJaKJz4Ks7wmAPeQv_Kr 
#undef  mwNI1C9jk_3pvJZrX0fHS 
#undef  muCpTNCg7lxg5KESlCXG4 
#undef  mcyd2SKTOMs686Zg6AvKk 
#undef  mjdc8VjjXr0Aa4BhmKceu 
#undef mptGld_s1__181L7c5bqePhCNciRpVt
#undef  myST3yoY8lduEVp56TZvE 
#undef  mrBBhc8z_AaoJbohTIyYv 
#undef  mZWWgl3f4HYSI9eS8vy0R 
#undef  mCqKRJorgbRSota6iXCJi 
#undef  mRVNu3uBifTB8BG7OUpAW 
#undef  mxgClYaoWf3JD92DcY_Nu 
#undef  maVKvXclRsbYDWsFCCRQ6 
#undef mIj9RxIlpb3DDc43lkNYOW4b9xAtXRt
#undef  mcL_jnyjEI8_Q9GUQt879 
#undef  mqo54nGxH5tgGNB_7gzrS 
#undef  mLoK8iXU7kkTTdrmwXZxZ 
#undef  mG2qGkHXgoSG0kBsCveUo 
#undef  mpHnjh3bjrfzaVGTos50i 
#undef  mi0QF0oVhhyvg9CkdWFG3 
#undef  mHQ_vtj7P_ADaccfNy6mJ 
#undef  mvY_F7NMDRsxqUYYdXTuC 
#undef  mtMYa5oQ0r8zESiyoRoz5 
#undef  mr1IbghXlRrSngYASFFEI 
#undef  mxnHnBAlV1zBx0pFgG2hE 
#undef mYAITbC7HcOzUURw1ql9pDQP9jaXbuG
#undef mT97Vl3mfMFxypXAGSHk_jZ5UD7_1rX
#undef  mc4LhKWumx_776hOW_Kjl 
#undef  milrf8w3_M7vOm9VF2oue 
#undef  mO4LhRb5c45Kiu3TA0qLs 
#undef  mERHIaA8dcuwR0EcsP11Q 
#undef  mxTCLR2zVLUo76oVRUsFY 
#undef  meV7YtVCPInvXhLXG7Pps 
#undef  mYs9r6cAENDkSxxWuSiaO 
#undef  mZS9ceGnq7lgqx621Pk0t 
#endif
