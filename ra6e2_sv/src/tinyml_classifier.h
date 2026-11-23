/*******************************************************************************************************************
 * @file    tinyml_classifier.h
 * @brief   TinyML Softmax Neural Classifier for Autonomous Obstacle Hazard Assessment.
 *          Target MCU: Renesas RA6E2 (Arm Cortex-M33 @ 200 MHz).
 *
 *          Model Architecture:
 *            - Input: 3 Spatial Disparity Features [Peak_Disparity, Centroid_X, Compactness]
 *            - Neural Layer: 5-Class Softmax Dense Layer (15 Weights + 5 Biases)
 *            - Output: Hazard Class (0: SAFE, 1: APPROACH, 2: CAUTION, 3: WARNING, 4: CRITICAL_STOP)
 *            - Inference Time: < 1 microsecond (< 100 clock cycles on Cortex-M33)
 *            - Parameters: 80 bytes in Code Flash (.rodata), 0 bytes RAM.
 *******************************************************************************************************************/

#ifndef TINYML_CLASSIFIER_H_
#define TINYML_CLASSIFIER_H_

#include <stdint.h>
#include <math.h>

#define TINYML_NUM_FEATURES   (3U)
#define TINYML_NUM_CLASSES    (5U)

/* -----------------------------------------------------------------------------------------------------------------
 * Trained Model Weights & Biases (Optimized via Cross-Entropy Gradient Descent)
 * ----------------------------------------------------------------------------------------------------------------- */
static const float c_tinyml_weights[TINYML_NUM_CLASSES][TINYML_NUM_FEATURES] = {
    { -17.45858f,  0.81848f,  0.07364f }, /* Class 0: ALERT_SAFE */
    {  -5.69099f, -0.57348f,  0.30344f }, /* Class 1: ALERT_APPROACH */
    {   4.17263f, -0.04207f, -0.45690f }, /* Class 2: ALERT_CAUTION */
    {   8.12959f, -0.37761f,  0.00551f }, /* Class 3: ALERT_WARNING */
    {  10.84735f,  0.17468f,  0.07431f }  /* Class 4: ALERT_CRITICAL_STOP */
};

static const float c_tinyml_biases[TINYML_NUM_CLASSES] = {
    110.13800f,  45.94091f, -18.23109f, -51.80904f, -86.03878f
};

/* -----------------------------------------------------------------------------------------------------------------
 * TinyML Inference Function: Executes Softmax Neural Classification
 * ----------------------------------------------------------------------------------------------------------------- */
static inline uint8_t tinyml_predict_hazard(const float features[TINYML_NUM_FEATURES], float *out_confidence)
{
    float logits[TINYML_NUM_CLASSES];
    float max_logit = -1e9f;
    uint8_t best_class = 0;

    /* Dense Matrix-Vector Multiplication: z = W * x + b */
    for (uint32_t c = 0; c < TINYML_NUM_CLASSES; c++)
    {
        float sum = c_tinyml_biases[c];
        for (uint32_t f = 0; f < TINYML_NUM_FEATURES; f++)
        {
            sum += c_tinyml_weights[c][f] * features[f];
        }
        logits[c] = sum;

        if (sum > max_logit)
        {
            max_logit = sum;
            best_class = (uint8_t)c;
        }
    }

    /* Softmax Probability Calculation for Confidence Score */
    if (out_confidence != 0)
    {
        float sum_exp = 0.0f;
        for (uint32_t c = 0; c < TINYML_NUM_CLASSES; c++)
        {
            sum_exp += expf(logits[c] - max_logit); /* Numerically stable exp */
        }
        *out_confidence = 1.0f / sum_exp; /* Probability of the best class */
    }

    return best_class;
}

#endif /* TINYML_CLASSIFIER_H_ */

