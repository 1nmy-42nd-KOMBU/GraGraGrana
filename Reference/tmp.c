define P 5 //比例ゲイン(P制御の操作量調整用)

define D 0.55 //微分ゲイン(D制御の操作量調整用)

define zensin 10000 //前進するためにMtr_Run_lv関数に入れる値

define dt 0.02 //過去から現在までの時間（制御周期）

int main (void)
{
int data0=0; //左赤外線センサ値格納用
int data1=0; //右赤外線センサ値格納用
int sousaryo; //操作量格納用
int hensa; //偏差格納用
int kako=0; //過去の偏差格納用
double bibun;

//制御周期の設定[単位：Hz　範囲：30.0~]
const unsigned short MainCycle = 60;
Init(MainCycle);                     //CPUの初期設定
while(getSW()==0){}                    //ボタンを押したらスタート
while(getSW()==1){}
LED(1);

while(1){                         //{}内を永遠に繰り返す
data0=ADRead(0);                //左赤外線センサ値格納
data1=ADRead(1);                //右赤外線センサ値格納
hensa=data0-data1-10;               //偏差を求める式、10は赤外線センサの個体差により違うため注意。
bibun=(double)(hensa-kako)/dt;          //Dの操作量を求める式
sousaryo=P*hensa+D*bibun;           //hensaは値が小さすぎるからP倍し、Dは値が大きすぎるからD倍している
if(sousaryo>16000 ){sousaryo=16000;}        //操作量が大きくなりすぎないように最大でも16000にしている
if(sousaryo<-16000 ){sousaryo=-16000;}      //操作量が小さくなりすぎないように最大でも16000にしている
Mtr_Run_lv(-sousaryo+zensin,-sousaryo-zensin,0,0,0,0);  
                        //モータに旋回の値（hensa）と前進の値(zensin)を与える
    Wait(20);//0.02秒待つ                //0.02秒待つ
    kako=hensa;                    //現在のhensaをkakoに記録する（D制御用）
}
}