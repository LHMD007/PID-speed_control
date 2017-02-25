# PID-speed_control
  increasing PID control algorithm，encoder,speed_control
//本人所用车模是C车模，双电机，电机控制采用增量式PID控制做速度闭环,
//控制频率为20ms一次
//车模左右电机有些许差异，速度快了之后PID参数不能共用，故应设置两套PID参数，
//加入鲁棒控制，也可以加入积分分离环节进行调控，由实际调试的效果看不出来积分分离
//在这里能起太大的作用。
