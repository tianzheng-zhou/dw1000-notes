#include <Arduino.h>
#include <DW1000Ranging.h>
#include <DW1000.h>
#include <DW1000Device.h>
#include <vector>

#include <iostream>

#include <cmath>


struct Coordinate {
  double x;
  double y;
};

class DW1000CoordinateTwoDimension  {

private:

  std::vector<double> m_range;
  std::vector<double> x_axis;
  std::vector<double> y_axis;

  // 屎山 不要动
  void matrixInverse2x2(const double A[2][2], double inv[2][2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if(fabs(det) < 1e-10) { // 防止奇异矩阵
        inv[0][0] = inv[1][1] = 0;
        inv[0][1] = inv[1][0] = 0;
        return;
    }
    double inv_det = 1.0 / det;
    inv[0][0] =  A[1][1] * inv_det;
    inv[0][1] = -A[0][1] * inv_det;
    inv[1][0] = -A[1][0] * inv_det;
    inv[1][1] =  A[0][0] * inv_det;
}
void matrixMultiply(const double* A, const double* B, double* C, int m, int n, int p) {
  for(int i=0; i<m; i++){
      for(int j=0; j<p; j++){
          C[i*p + j] = 0;
          for(int k=0; k<n; k++){
              C[i*p + j] += A[i*n + k] * B[k*p + j];
          }
      }
  }
}
Coordinate trilaterate(const Coordinate anchors[3], const double distances[3]) {
  double x1 = anchors[0].x, y1 = anchors[0].y;
  double x2 = anchors[1].x, y2 = anchors[1].y;
  double x3 = anchors[2].x, y3 = anchors[2].y;
  double d1 = distances[0], d2 = distances[1], d3 = distances[2];

  // 构建矩阵A (3x2)
  double A[3][2] = {
      {2*(x2 - x1), 2*(y2 - y1)},
      {2*(x3 - x1), 2*(y3 - y1)},
      {2*(x3 - x2), 2*(y3 - y2)}
  };

  // 构建向量b (3x1)
  double b[3] = {
      d1*d1 - d2*d2 + x2*x2 + y2*y2 - x1*x1 - y1*y1,
      d1*d1 - d3*d3 + x3*x3 + y3*y3 - x1*x1 - y1*y1,
      d2*d2 - d3*d3 + x3*x3 + y3*y3 - x2*x2 - y2*y2
  };

  // 计算 A^T * A 和 A^T * b
  double AT[2][3]; // 转置矩阵
  for(int i=0; i<3; i++) {
      AT[0][i] = A[i][0];
      AT[1][i] = A[i][1];
  }

  // 计算 A^T * A (2x2)
  double ATA[2][2];
  matrixMultiply((double*)AT, (double*)A, (double*)ATA, 2, 3, 2);

  // 计算 A^T * b (2x1)
  double ATb[2];
  matrixMultiply((double*)AT, b, ATb, 2, 3, 1);

  // 求逆矩阵 (A^T A)^-1
  double invATA[2][2];
  matrixInverse2x2(ATA, invATA);

  // 计算最终解 x = (A^T A)^-1 * A^T b
  double x[2];
  matrixMultiply((double*)invATA, ATb, x, 2, 2, 1);

  return {x[0], x[1]};
}




public:

/*用于添加设备
 *@param device 设备
 *@param x 设备在x轴上的坐标
 *@param y 设备在y轴上的坐标
 *
*/
  void addDevice(DW1000Device* device, double x, double y) {
    m_range.push_back(device->getRange());
    x_axis.push_back(x);
    y_axis.push_back(y);
  }

  /*用于添加设备
  *@param range 设备到基站的距离
  *@param x 设备在x轴上的坐标
  *@param y 设备在y轴上的坐标
  *
  */
  void addDevice(double range, double x, double y) {
    m_range.push_back(range);
    x_axis.push_back(x);
    y_axis.push_back(y);
  }
  
  /*
  *用于计算设备的坐标
  *@return 设备的坐标 数据类型为Coordinate
  */
  Coordinate calculate_coordinate() {
    
    // 屎山 不要动
    if(m_range.size() == 3) {

      Coordinate anchors[3] = {{x_axis[0], y_axis[0]}, {x_axis[1], y_axis[1]}, {x_axis[2], y_axis[2]}};
      double distances[3] = {m_range[0], m_range[1], m_range[2]};

      Coordinate result=trilaterate(anchors, distances);

      return result;

    }

  }

};

void setup() {
  Serial.begin(9600);
}

void loop() {

  DW1000CoordinateTwoDimension c;

  c.addDevice(1.4, 0.0, 0.0);
  c.addDevice(1.0, 1.0, 0.0);
  c.addDevice(1.0, 0.0, 1.0);
  Coordinate result = c.calculate_coordinate();
  Serial.print("x: ");
  Serial.println(result.x);
  Serial.print("y: ");
  Serial.println(result.y);
  delay(1000);
  
}
