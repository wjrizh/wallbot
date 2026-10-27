// CAP 采集结束后的最终预测入口（给 ae_uart_service 调用）。
// 与网页 live_prediction 不同：以 .pridb 事件特征为主，不因现场波形
// 点数≠200000/20000 而把距离清空（桌面端/旧脚本常见行为）。
'use strict';

const { loadModel, predictOne } = require('./predict_distance_10.20.js');

function predict(pridbPath) {
  return predictOne(pridbPath, loadModel(), { allowIncompatibleTradb: true });
}

if (require.main === module) {
  const file = process.argv[2];
  if (!file) {
    console.error('用法: node cap_prediction.js <采集.pridb>');
    process.exitCode = 2;
  } else {
    try {
      console.log(JSON.stringify(predict(file)));
    } catch (error) {
      console.error(error.message || error);
      process.exitCode = 1;
    }
  }
}

module.exports = { predict };
