"""
MicroEdgeDepth - End-to-End Trainer & INT8 TFLite Exporter for Renesas RA6E2
Trains on 64x64 synthetic stereo pairs and exports directly to `model_data.h`
compatible with Renesas e2 studio and Arm CMSIS-NN.

Usage:
  py -3.11 training_setup/train_and_export.py
"""

import os
import random
import numpy as np
import tensorflow as tf

# Suppress verbose TF logging
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

IMG_SIZE = 64
NUM_DISP = 4       # 4 hypotheses at 16x16 (0, 4, 8, 12 px at full scale)
GROUPS   = 4       # 4 cost channels per disparity hypothesis

# ==============================================================================
# 1. Dataset Generation (Synthetic Stereo Pairs on the fly)
# ==============================================================================
def generate_stereo_batch(batch_size=32, img_size=64, max_disp=12):
    left_imgs  = np.zeros((batch_size, img_size, img_size, 1), dtype=np.float32)
    right_imgs = np.zeros((batch_size, img_size, img_size, 1), dtype=np.float32)
    disp_gts   = np.zeros((batch_size, img_size, img_size, 1), dtype=np.float32)

    for b in range(batch_size):
        # Textured background
        left = np.random.uniform(0.1, 0.4, (img_size, img_size))
        noise = np.random.normal(0, 0.05, (img_size, img_size))
        left = np.clip(left + noise, 0.0, 1.0)
        right = left.copy()
        gt = np.zeros((img_size, img_size), dtype=np.float32)

        # Draw 2 to 4 random foreground objects
        for _ in range(random.randint(2, 4)):
            d = random.randint(1, max_disp - 1)
            ow, oh = random.randint(10, 20), random.randint(10, 20)
            x0 = random.randint(d, img_size - ow - 1)
            y0 = random.randint(0, img_size - oh - 1)

            patch = np.clip(np.random.uniform(0.5, 0.95, (oh, ow)) + np.random.normal(0, 0.05, (oh, ow)), 0.0, 1.0)
            left[y0:y0+oh, x0:x0+ow] = patch
            gt[y0:y0+oh, x0:x0+ow] = float(d)
            right[y0:y0+oh, x0-d:x0-d+ow] = patch

        # Normalize to standard [-1.0, 1.0] range
        left_imgs[b, ..., 0]  = (left - 0.5) / 0.5
        right_imgs[b, ..., 0] = (right - 0.5) / 0.5
        disp_gts[b, ..., 0]   = gt

    return left_imgs, right_imgs, disp_gts

# ==============================================================================
# 2. MicroEdgeDepth Model Definition in Keras / TensorFlow
# ==============================================================================
def build_micro_edgedepth():
    left_in  = tf.keras.Input(shape=(IMG_SIZE, IMG_SIZE, 1), name="left_image")
    right_in = tf.keras.Input(shape=(IMG_SIZE, IMG_SIZE, 1), name="right_image")

    # 1. Siamese Feature Extractor
    def make_tower():
        inp = tf.keras.Input(shape=(IMG_SIZE, IMG_SIZE, 1))
        # Stem: 64x64x1 -> 32x32x8
        x32 = tf.keras.layers.Conv2D(8, 3, strides=2, padding="same", use_bias=False)(inp)
        x32 = tf.keras.layers.BatchNormalization()(x32)
        x32 = tf.keras.layers.ReLU()(x32)
        # Feat16: 32x32x8 -> 16x16x12
        x16 = tf.keras.layers.DepthwiseConv2D(3, strides=2, padding="same", use_bias=False)(x32)
        x16 = tf.keras.layers.Conv2D(12, 1, strides=1, padding="same", use_bias=False)(x16)
        x16 = tf.keras.layers.BatchNormalization()(x16)
        x16 = tf.keras.layers.ReLU()(x16)
        return tf.keras.Model(inp, [x32, x16], name="siamese_tower")

    tower = make_tower()
    fl32, fl16 = tower(left_in)
    fr32, fr16 = tower(right_in)

    # 2. 2D Cost Volume Construction (Epipolar Shifts at 16x16)
    reduce_conv = tf.keras.layers.Conv2D(GROUPS, 1, strides=1, padding="same", use_bias=False, name="reduce_conv")
    cost_slices = []

    for d in range(NUM_DISP):
        if d == 0:
            shifted_r = fr16
        else:
            # Shift right features by padding left with zeros and slicing
            pad = tf.keras.layers.ZeroPadding2D(padding=((0, 0), (d, 0)))(fr16)
            shifted_r = tf.keras.layers.Cropping2D(cropping=((0, 0), (0, d)))(pad)

        diff = tf.keras.layers.Lambda(lambda pair: tf.abs(pair[0] - pair[1]))([fl16, shifted_r])
        reduced = reduce_conv(diff)
        cost_slices.append(reduced)

    cost_vol = tf.keras.layers.Concatenate(axis=-1)(cost_slices)  # 16x16x16

    # 3. Aggregation (Hourglass)
    x = tf.keras.layers.DepthwiseConv2D(3, padding="same", use_bias=False)(cost_vol)
    x = tf.keras.layers.Conv2D(16, 1, padding="same", activation="relu")(x)
    down = tf.keras.layers.MaxPool2D(2, 2)(x)                     # 8x8x16
    up = tf.keras.layers.UpSampling2D(2, interpolation="nearest")(down)
    x = tf.keras.layers.Add()([x, up])
    cost = tf.keras.layers.Conv2D(NUM_DISP, 3, padding="same", name="cost_logits")(x)  # 16x16x4

    # 4. Soft-Argmin (Fixed Ramp Kernel)
    prob = tf.keras.layers.Softmax(axis=-1)(-cost * 0.8)
    ramp_kernel = np.array([0.0, 4.0, 8.0, 12.0], dtype=np.float32).reshape(1, 1, 4, 1)
    ramp_conv = tf.keras.layers.Conv2D(1, 1, use_bias=False, 
                                       kernel_initializer=tf.constant_initializer(ramp_kernel),
                                       trainable=False, name="soft_argmin")
    disp16 = ramp_conv(prob)                                      # 16x16x1

    # 5. Guided Refinement to 32x32
    disp32_up = tf.keras.layers.UpSampling2D(2, interpolation="bilinear")(disp16)
    guide = tf.keras.layers.Concatenate(axis=-1)([disp32_up, fl32])  # 32x32x9
    res = tf.keras.layers.DepthwiseConv2D(3, padding="same", use_bias=False)(guide)
    res = tf.keras.layers.Conv2D(8, 1, padding="same", activation="relu")(res)
    res = tf.keras.layers.Conv2D(1, 3, padding="same")(res)
    disp32 = tf.keras.layers.ReLU()(tf.keras.layers.Add()([disp32_up, res]))

    # 6. Final Upsample to 64x64
    disp64 = tf.keras.layers.UpSampling2D(2, interpolation="bilinear", name="disparity_out")(disp32)

    return tf.keras.Model(inputs=[left_in, right_in], outputs=disp64, name="MicroEdgeDepth")

# ==============================================================================
# 3. Main Training & Export Loop
# ==============================================================================
def main():
    print("===============================================================")
    print(" MicroEdgeDepth: Automated Training & INT8 TFLite Export")
    print(" Target Hardware: Renesas FPB-RA6E2 (Arm Cortex-M33 @ 200 MHz)")
    print("===============================================================")

    model = build_micro_edgedepth()
    model.summary()

    # Compile with Huber loss (Smooth L1)
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=2e-3),
        loss=tf.keras.losses.Huber(delta=1.0),
        metrics=["mae"]
    )

    # Fast Training on 1,500 Synthetic Stereo Pairs
    print("\n[Step 1/3] Generating synthetic training data & training model...")
    train_l, train_r, train_gt = generate_stereo_batch(batch_size=1500, img_size=64, max_disp=12)
    val_l, val_r, val_gt       = generate_stereo_batch(batch_size=150,  img_size=64, max_disp=12)

    model.fit(
        x=[train_l, train_r],
        y=train_gt,
        validation_data=([val_l, val_r], val_gt),
        epochs=10,
        batch_size=32,
        verbose=1
    )

    # Representative Dataset for INT8 Calibration
    def rep_dataset():
        for i in range(100):
            yield [val_l[i:i+1].astype(np.float32), val_r[i:i+1].astype(np.float32)]

    # INT8 Post-Training Quantization (PTQ)
    print("\n[Step 2/3] Performing INT8 Post-Training Quantization (PTQ)...")
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = rep_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8

    tflite_model = converter.convert()
    tflite_path = "micro_edgedepth_int8.tflite"
    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    print(f"Quantized TFLite model saved: {tflite_path} ({len(tflite_model) / 1024:.2f} KB)")

    # Generate C Header Array for e2 studio
    print("\n[Step 3/3] Generating C Header 'model_data.h' for Renesas e2 studio...")
    header_path = "model_data.h"
    with open(header_path, "w") as f:
        f.write("#ifndef MODEL_DATA_H\n")
        f.write("#define MODEL_DATA_H\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write("// Stored in Code Flash (.rodata) -> Consumes 0 bytes of SRAM!\n")
        f.write(f"const unsigned int g_model_data_len = {len(tflite_model)};\n")
        f.write("const unsigned char g_model_data[] __attribute__((aligned(16), section(\".rodata\"))) = {\n")
        
        hex_lines = []
        for i, byte in enumerate(tflite_model):
            hex_lines.append(f"0x{byte:02x}")
            if (i + 1) % 12 == 0:
                f.write("    " + ", ".join(hex_lines) + ",\n")
                hex_lines = []
        if hex_lines:
            f.write("    " + ", ".join(hex_lines) + "\n")

        f.write("};\n\n")
        f.write("#endif // MODEL_DATA_H\n")

    print(f"C Header saved successfully: {os.path.abspath(header_path)}")
    print("\n===> ALL DONE! You can now copy 'model_data.h' into your e2 studio project.")

if __name__ == "__main__":
    main()
