## PID Controller Project

The goals / steps of this project are the following:

* Build a PID controller and tune the PID hyperparameters by applying the general processing flow as described in the previous lessons.

[//]: # (Image References)
[image1]: ./output_images/compilation.PNG
[image2]: ./output_images/manual_result.PNG
[image3]: ./output_images/auto_result.PNG
[video1]: ./output_videos/P_high.mp4
[video2]: ./output_videos/P_low.mp4
[video3]: ./output_videos/P_best.mp4
[video4]: ./output_videos/D_high.mp4
[video5]: ./output_videos/D_low.mp4
[video6]: ./output_videos/D_best.mp4
[video7]: ./output_videos/I_high.mp4
[video8]: ./output_videos/I_low.mp4
[video9]: ./output_videos/I_best.mp4
[video10]: ./output_videos/PID_ManualTuning.mp4
[video11]: ./output_videos/PID_AutoTuning.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Compilation

#### 1. Your code should compile.

![alt text][image1]

### Implementation

#### 1. The PID procedure follows what was taught in the lessons.

I implemented PID procedure follow lessons in file `PID.cpp`.
I also implemented some correction case when caculating `i_error` (Integral part):

* When the error finally reaches 0, the integral should be 0 to avoid the speed still high enough to keep the error chaning.
* By the time, the integral will wind-up and can be very large, far higher than what we need. So we need to limit the value of integral.

```cpp
  if (cte == 0)
  {
    i_error = 0;
  }
  if (fabs(cte) > 70)
  {
    i_error = 0;
  }
```

To tune PID gain easier, I implement 3 modes, it can be config in `CMakeList` file:

```
#-DMODE_NORMAL
#-DMODE_MANUAL
#-DMODE_TWIDDLE
```

##### 1.1 NORMAL MODE:

In this mode we can run binary file simply:

```bash
$ ./pid
```

All PID gain (Kp, Ki, Kd) are configured in source code, file `main.cpp` line 69.

```cpp
double p[3] = {0.2, 0.098, 0.08};
```

##### 1.2 MANUAL MODE:

In this mode we can run binary file with start point for tuning:

```bash
$ ./pid Kp_value Ki_value Kd_value
```

For example:

```bash
$ ./pid 0.2 0.098 0.08
```

Before run manual tuning, we have to modify source, file `main.cpp` line 262, to choose what gain we want to tune.
For example we want to tune Ki:

```cpp
p[1] += 0.001;
```

##### 1.1 TWIDDLE MODE:

In this mode we can run binary file with start point for tuning:

```bash
$ ./pid Kp_value Ki_value Kd_value
```

For example:

```bash
$ ./pid 0.2 0.098 0.08
```

Before run auto tuning, we have to modify source, file `main.cpp` line 77, to choose what start point for potential changes.
For example:

```cpp
double dp[3] = {0.1, 0.001, 0.01};
```

Twiddle function is implemented in file `main.cpp` line 85-144.

### Reflection

#### 1. Describe the effect each of the P, I, D components had in your implementation.

##### 1.1 P controller: controls the cross track error (CTE) smoothly.

When CTE is large, it help reduce CTE quickly.
Until CTE is small, it reduce CTE slow down.

* P gain is too large will make our system unstable:

Here's a [link to my video test with Kp is 1.0][video1]

* P gain is too small will make our system weak:

Here's a [link to my video test with Kp is 0.01][video2]

* P gain is suitable will make our system stable:

Here's a [link to my video test with Kp is 0.2][video3]

##### 1.2 D controller: predict the future value of the error, reduce overshot.

* D gain is too large will make our system reduce time response but will make it unstable:

Here's a [link to my video test with Kd is 10.0][video4]

* D gain is too small will have too few effect to system:

Here's a [link to my video test with Kd is 0.01][video5]

* D gain is suitable will make our system stable with best time response and minimal overshot:

Here's a [link to my video test with Kd is 0.08][video6]

##### 1.3 I controller: makes system stable by reduce overshot and closest setpoint in case the error too small.

* I gain is too large will make our system unstable:

Here's a [link to my video test with Ki is 1.0][video7]

* I gain is too small will have too few effect to system:

Here's a [link to my video test with Ki is 0.001][video8]

* I gain is suitable will make our system stable with best time response and minimal overshot:

Here's a [link to my video test with Ki is 0.08][video9]

#### 2. Describe how the final hyperparameters were chosen.

Firstly, I tuned PID gain manually to help the car can successfully drive a lap around the track with as fast as possible:

* Init all Kp, Ki, Kd to 0
* Tune Kp: I test with a large Kp to make the car run in lane with a sine wave with minimum cycle. And I get value 1.
After that, I reduce Kp by 0.1 step by step and see what changing.
When value is 0.2, I see that the car can run with a sine wave with acceptable cycle, car can run as far as it can. Ofcourse it still have overshot.
And I choose Kp is 0.2.
* Tune Kd: To make the controller reach setpoint (move into center lane) faster and reduce overshot I incease Kd by 0.01 step by step from start point is 0.01.
And I choose Kd is 0.08.
* Tune Ki: To reduce overshot I incease Ki by 0.01 step by step from start point is 0.01 to see how it reduce overshot maked by Kp.
And I choose Ki is 0.08.

So I get a good PID gain:

| Gain | Value |
|:----:|:-----:|
| Kp   | 0.2   |
| Ki   | 0.08  |
| Kd   | 0.08  |

![alt text][image2]

Total error is 1491.72.
Here's a [link to my video result for manual tuning][video10]

Finally, I use Twiddle algorithm to tune PID gain again to get better result.
So I get a best PID gain:

| Gain | Value     |
|:----:|:---------:|
| Kp   | 0.389311  |
| Ki   | 0.0961499 |
| Kd   | 0.184516  |

![alt text][image3]

Total error is smaller, 1209.46.
Here's a [link to my video result for auto tuning][video11]

### Simulation

#### 1. The vehicle must successfully drive a lap around the track.

My Twiddle algorithm maybe is not good, so it make car run smoothly when the lane have little curve. When the curve angle is large, the result is not good.

So finally I choose the result of manual tuning.

Here's a [link to my final video result][video10]

---

### References:

* https://udacity-reviews-uploads.s3.amazonaws.com/_attachments/41330/1493863065/pid_control_document.pdf
* https://discussions.udacity.com/t/how-to-tune-parameters/303845/4
* https://discussions.udacity.com/t/how-to-implement-twiddle-optimisation/279749/2
* https://discussions.udacity.com/t/how-to-twiddle-in-pid-controller-project/246971
* https://discussions.udacity.com/t/how-do-we-verify-our-implementation-of-twiddle-is-correct/295819/2
* https://martin-thoma.com/twiddle/

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.

##### Techniques:

* Manual tuning by experience
* Auto tuning by Twiddle Algorithm

##### Fail cases:

* The speed is too high (>50mph), my controller maybe cannot adaptive.

##### Improve:

* Implement PID speed to control speed with CTE together.
* Improve Twiddle function
