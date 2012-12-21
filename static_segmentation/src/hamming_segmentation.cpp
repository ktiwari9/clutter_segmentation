#include <opencv2/opencv.hpp>
#include <vector>
#include <set>
#include <sys/time.h>
#include <ros/ros.h>

using namespace std;

// Produce [numPlanes] random vectors, each of dimension [numDimensions].
vector<float> makeRandomPlanes(int numPlanes, int numDimensions) {
  vector<float> planes(numPlanes*numDimensions);

  for(int i = 0; i < numPlanes * numDimensions; i++)
      planes[i] = ((rand() / (float)RAND_MAX) - 0.5) * 2.0;

  return planes;
}

float timeDiff(struct timeval& start, struct timeval& stop) {
  return (stop.tv_sec - start.tv_sec) + \
         0.000001f * (stop.tv_usec - start.tv_usec);
}

inline uint32_t hammingDistance(uint32_t x, uint32_t y)
{
    uint32_t dist = 0;
    uint32_t r = x ^ y;
    for(dist = 0; r; dist++) r &= r - 1;
    return dist;
}

// Compute per-pixel projections
inline void projectPixels(const cv::Mat& imgIn,
                          const vector<float>& planes,
                          float* projections,
                          vector<float>& minimums,
                          vector<float>& maximums) {
  int numChannels = imgIn.channels();
  int numPlanes = planes.size() / numChannels;
  float** minRows = (float**)malloc(sizeof(float*)*imgIn.rows);
  float** maxRows = (float**)malloc(sizeof(float*)*imgIn.rows);
  
  #pragma omp parallel for
  for(int y = 0; y < imgIn.rows; y ++) {
    const uchar* row = imgIn.ptr(y);
    //vector<float> pixelBuffer(numChannels);
    // HACK: we almost always have 3 color channels
    float pixelBuffer[4];
    float* minRow = (float*)malloc(sizeof(float)*numPlanes);
    memset(minRow, 0, sizeof(float)*numPlanes);
    float* maxRow = (float*)malloc(sizeof(float)*numPlanes);
    memset(maxRow, 0, sizeof(float)*numPlanes);

    float* projPtr = &projections[y*numPlanes*imgIn.cols];
    for(int x = 0, p = 0; x < imgIn.cols; x++) {
      for(int d = 0; d < numChannels; d++, row++) 
        pixelBuffer[d] = (float)(*row) - 128.0f;

      const float* planePtr = &planes[0];
      for(int plane = 0; plane < numPlanes; plane++) {
        float sum = 0.0f;
        for(int d = 0; d < numChannels; d++, planePtr++)
          sum += pixelBuffer[d] * (*planePtr);
        *(projPtr++) = sum;
        if(sum > maxRow[plane]) maxRow[plane] = sum;
        if(sum < minRow[plane]) minRow[plane] = sum;
      }
    }
    minRows[y] = minRow;
    maxRows[y] = maxRow;
  }
  
  memcpy(&(minimums[0]), minRows[0], sizeof(float)*numPlanes);
  memcpy(&(maximums[0]), maxRows[0], sizeof(float)*numPlanes);
  free(minRows[0]);
  free(maxRows[0]);

  for(int y = 1; y < imgIn.rows; y++) {
    float* minRow = minRows[y];
    float* maxRow = maxRows[y];
    for(int plane = 0; plane < numPlanes; plane++) {
      if(minRow[plane] < minimums[plane])
        minimums[plane] = minRow[plane];
      if(maxRow[plane] > maximums[plane])
        maximums[plane] = maxRow[plane];
    }
    free(minRow);
    free(maxRow);
  }
  free(minRows);
  free(maxRows);
}

// Compute the midpoint of the extrema of the projections for each
// plane.
inline void encodeProjections(const vector<float>& minimums,
                              const vector<float>& maximums,
                              const float* projections,
                              cv::Mat& imgOut,
                              vector<uint32_t>& bins) {
  int numPlanes = minimums.size();
  //vector<float> midpoints(numPlanes);
  float midpoints[numPlanes];
  for(int plane = 0; plane < numPlanes; plane++)
    midpoints[plane] = (maximums[plane]+minimums[plane]) * 0.5f;

  // Encode the per-pixel projections using midpoint information.
  for(int y = 0; y < imgOut.rows; y++) {
    //int* row = imgOut.ptr(y);
    uint32_t* row = ((uint32_t*)imgOut.data) + y*imgOut.cols;
    const float* projRow = &(projections[y * imgOut.cols * numPlanes]);
    for(int x = 0; x < imgOut.cols; x++, row++) {
      uint32_t code = 0;
      for(uint32_t plane = 0, mask = 1; 
          plane < numPlanes; 
          plane++, mask *= 2, projRow++) {
        if(*projRow > midpoints[plane]) code |= mask;
      }
      *row = code;
      bins[code]++;
    }
  }
}

// Compute local maxima in Hamming space
inline void hammingMaxima(const vector<uint32_t>& bins,
                          int numPlanes,
                          int hammingK,
                          vector<uint32_t>& hMaxima) {
  for(int i = 0; i < bins.size(); i++) {
    int myPop = bins[i];
    bool ismax = true;
    if(myPop == 0) continue;
    for(int j = 0; j < hammingK; j++) {
      int neighbor = 1 << (numPlanes - 1);
      for(int k = 0; k < j; k++)
        neighbor |= 1 << (numPlanes - 2 - k);

      while(neighbor > (1 << j)) {
        if(bins[neighbor ^ i] >= myPop) {
          j = hammingK;
          ismax = false;
          break;
        }
        neighbor = neighbor >> 1;
      }
    }
    if(ismax) hMaxima.push_back(i);
  }
}

// Compute a mapping from each present Hamming code to a local maximum.
void mapToMaxima(const vector<uint32_t>& bins,
                 const vector<uint32_t>& hMaxima,
                 uint32_t* binMapping) {
  set<uint32_t> maximaSet(hMaxima.begin(), hMaxima.end());

  // Map the set of maxima to the naturals
  for(int i = 0; i < hMaxima.size(); i++) {
    binMapping[hMaxima[i]] = i;
  }

  for(int i = 0; i < bins.size(); i++) {
    if(bins[i] == 0) continue;
    if(maximaSet.find(i) == maximaSet.end()) {
      // Find the nearest Hamming maximum
      uint32_t minDist = hammingDistance(hMaxima[0], i);
      int bestCenter = 0;
      for(int b = 1; b < hMaxima.size(); b++) {
        uint32_t dist = hammingDistance(hMaxima[b], i);
        if(dist < minDist) {
          minDist = dist;
          bestCenter = b;
        }
      }
      binMapping[i] = binMapping[hMaxima[bestCenter]];
    }
  }
}

void hammingHash(const cv::Mat& imgIn, cv::Mat& imgOut,
                 const vector<float>& planes,
                 uint32_t hammingK) {
  imgOut.create(imgIn.rows, imgIn.cols, CV_32S);
  int numChannels = imgIn.channels();
  int numPlanes = planes.size() / numChannels;
  vector<float> minimums(numPlanes);
  vector<float> maximums(numPlanes);

  //vector<float> projections(imgIn.rows*imgIn.cols*numPlanes);
  float* projections = 
    (float*)malloc(sizeof(float)*imgIn.rows*imgIn.cols*numPlanes);

  // Project pixels onto the given planes. This is effected by
  // considering the sign of the dot product between each pixel and
  // the vector associated with each plane.
  projectPixels(imgIn, planes, projections, minimums, maximums);

  // Generate a binary encoding of each projection, store the codes in imgOut.
  vector<uint32_t> bins(1 << numPlanes, 0);
  encodeProjections(minimums, maximums, projections, imgOut, bins);

  free(projections);

  // Compute local maxima in Hamming space
  vector<uint32_t> hMaxima;
  hammingMaxima(bins, numPlanes, hammingK, hMaxima);
  if(hMaxima.size() < 1) return;

  cout << "Found " << hMaxima.size() << " local maxima" << endl;

  // Map non-maxima in Hamming space to nearest maximum.
  uint32_t* binMapping = (uint32_t*)malloc(sizeof(uint32_t)*bins.size());
  mapToMaxima(bins, hMaxima, binMapping);

  // Apply the mapping to our coded image
  for(int y = 0; y < imgIn.rows; y++) {
    int* row = (int*)(imgOut.data) + y*imgIn.cols;
    for(int x = 0; x < imgIn.cols; x++, row++) {
      *row = binMapping[*row];
    }
  }
  free(binMapping);
}

int main(int argc, char** argv) {
  if(argc != 2) {
    cout << "Usage: ./segment imgFile" << endl;
    return 1;
  }

  // We're using a colorspace with 3 channels, and using 6 splitting
  // planes.
  vector<float> planes = makeRandomPlanes(6,3);
  
  cv::Mat imgIn = cv::imread(argv[1]);
  cv::Mat imgHSV, imgCode;
  cv::cvtColor(imgIn, imgHSV, CV_BGR2HSV);

  cout << "Hold on to your butts..." << endl;
  struct timeval start, stop;
  gettimeofday(&start, NULL);
  hammingHash(imgHSV, imgCode, planes, 1);
  gettimeofday(&stop, NULL);
  printf("Hamming hash took %.1fms\n", timeDiff(start,stop)*1000.0f);
  imwrite("coded.png", imgCode);
  return 0;
}
