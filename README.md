# 3-axis Attitude Control Simulation for MATLAB
簡易な剛体モデルを使った三軸姿勢制御，三軸姿勢推定シミュレータです． 
真空中，無重力化を想定しており環境外乱は考慮していません． 

## デモ
https://user-images.githubusercontent.com/62766332/222377441-ffadc8ef-792d-435e-8d1b-cfe0a934deb4.mp4

## ファイル説明
### cube_rot_sim.m  
メインmatlabファイル，基本的な三軸姿勢制御と三軸姿勢推定のシミュレーションが行えます．  
出力は動画(mp4)で出力します．

### est_filter_lib
フィルタライブラリ
* complementaryFilter.m  
  相補フィルタ
  
* getAngle.m  
  フィルタ無しでオイラー角を計算
  
* jacobiMat.m  
  ヤコビ行列を求める
  
* kalmanFilter9Axis.m  
  9軸センサ用カルマンフィルタ 
  
* kalmanFilter9AxisWithBias.m  
  バイアス補正有り9軸センサ用カルマンフィルタ(動作しない)
  
* lowpassFilterFo.m  
  一次ローパスフィルタ
  
* madgwickFilter9Axis.m  
  9軸センサ用マドウィックフィルタ

### quaternion_lib
クォータニオンライブラリ
* euler2Q.m  
  オイラー角からクォータニオンに変換
  
* euler2Rotmat.m  
  オイラー角から回転行列に変換
  
* q2Euler.m  
  クォータニオンからオイラー角に変換
  
* q2Rotmat.m  
  クォータニオンから回転行列に変換
  
* qConj.m  
  クォータニオンの共役(逆元)を求める
  
* qCross.m  
  クォータニオン積を求める
  
* qCrossMat.m  
  クォータニオン積の表現行列を求める
  
* qm2Qu.m  
  unity座標系のクォータニオンに変換
  
* qNormalize.m  
  クォータニオンの正規化を行う
  
* qr2Ql.m  
  右手系から左手系にクオータニオンを変換
  
* qS.m  
  クォータニオンのスカラー部を取り出す
  
* qV.m  
  クォータニオンのベクトル部を取り出す
  
* rotmat2Euler.m  
  回転行列からオイラー角を求める
  
* rotmat2Q.m  
  回転行列からクォータニオンを求める
  
* vCrossMat.m  
  クォータニオン積の行列表現を求める (入力がベクトル)
  
* vRotation.m  
  回転させたベクトルを求める

### unity_com_lib
unity通信ライブラリ (Instrument Control Toolboxが必要)

* unityWrite.m  
  unity側に文字列データを転送
  
* unitySetup.m  
  tcpipクライアントオブジェクトの取得 
