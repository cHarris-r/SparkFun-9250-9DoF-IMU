
/*************************************************
** Vector_Dot_Product
** Computes the dot product of two vectors 
*/
float Vector_Dot_Product(const float v1[3], const float v2[3])
{
  float result = 0;
  for(int c = 0; c < 3; c++) { result += v1[c] * v2[c]; }
  return result; 
}


/*************************************************
** Vector_Cross_Product
** Computes the cross product of two vectors 
*/
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}


/*************************************************
** Vector_Scale
** Multiply a vector by a scalar (element-wise)
** Vector_Scale( B, A, c)
** B = A.*c
*/
void Vector_Scale(float out[3], const float v[3], float scale)
{
  for(int c = 0; c < 3; c++) { out[c] = v[c] * scale; }
}


/*************************************************
** Vector_Add
** Adds two vectors 
** Vector_Add( C, A, B )
** C = A + B 
*/
void Vector_Add(float out[3], const float v1[3], const float v2[3])
{
  for(int c = 0; c < 3; c++) { out[c] = v1[c] + v2[c]; }
}


/*************************************************
** Matrix_Multiply
** Multiply two 3x3 matrices: 
** Matrix_Multiply( out, a, b )
** out = a * b 
*/
void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3])
{
  for(int x = 0; x < 3; x++)  // rows
  {
    for(int y = 0; y < 3; y++)  // columns
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}


/*************************************************
** Multiply 3x3 matrix with 3x1 vector
** Outputs a 3x1 vector
** Matrix_Vector_Multiply( a[][], b[], out[] ) 
** out = a * b 
*/
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++) { out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2]; }
}


/*************************************************
** f_asin
** A faster arcsin
*/
float f_asin(float x) {
  float negate = float(x < 0);
  x = abs(x);
  float ret = -0.0187293;
  ret *= x;
  ret += 0.0742610;
  ret *= x;
  ret -= 0.2121144;
  ret *= x;
  ret += 1.5707288;
  ret = 3.14159265358979*0.5 - sqrt(1.0 - x)*ret;
  return ret - 2 * negate * ret;
}


/************************************************* 
** f_atan2
** A faster arctan2
*/
float f_atan2(float y, float x)
{
  float t0, t1, t2, t3, t4;

  t3 = abs(x);
  t1 = abs(y);
  t0 = max(t3, t1);
  t1 = min(t3, t1);
  t3 = float(1) / t0;
  t3 = t1 * t3;

  t4 = t3 * t3;
  t0 = - float(0.013480470);
  t0 = t0 * t4 + float(0.057477314);
  t0 = t0 * t4 - float(0.121239071);
  t0 = t0 * t4 + float(0.195635925);
  t0 = t0 * t4 - float(0.332994597);
  t0 = t0 * t4 + float(0.999995630);
  t3 = t0 * t3;

  t3 = (abs(y) > abs(x)) ? float(1.570796327) - t3 : t3;
  t3 = (x < 0) ?  float(3.141592654) - t3 : t3;
  t3 = (y < 0) ? -t3 : t3;

  return t3;
}









