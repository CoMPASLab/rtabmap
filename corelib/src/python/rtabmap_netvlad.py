#! /usr/bin/env python3
#
# Drop this file in the "python" folder of NetVLAD git (tensorflow-v1 used): https://github.com/uzh-rpg/netvlad_tf_open/
# Updated to work with https://github.com/uzh-rpg/netvlad_tf_open/pull/9
# To use with rtabmap:
#   --Mem/GlobalDescriptorStrategy 1 --Kp/TfIdfLikelihoodUsed false --Mem/RehearsalSimilarity 1 --PyDescriptor/Dim 128 --PyDescriptor/Path ~/netvlad_tf_open/python/rtabmap_netvlad.py
#

import sys    
import os
import numpy as np
import time
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
if not hasattr(sys, 'argv'):
    sys.argv  = ['']
    
#print(os.sys.path)
#print(sys.version)

import tensorflow as tf
import netvlad_tf.net_from_mat as nfm
import netvlad_tf.nets as nets

image_batch = None
net_out = None
saver = None
sess = None
dim = 4096

def init(descriptorDim):
    print("NetVLAD python init()")
    global image_batch
    global net_out
    global saver
    global sess
    global dim
    
    try:
        dim = descriptorDim

        tf.compat.v1.disable_eager_execution()
        tf.compat.v1.reset_default_graph()

        print("Creating TensorFlow placeholder...")
        image_batch = tf.compat.v1.placeholder(
            dtype=tf.float32, shape=[None, None, None, 3])

        print("Creating NetVLAD network...")
        net_out = nets.vgg16NetvladPca(image_batch)
        
        print("Creating TensorFlow saver...")
        saver = tf.compat.v1.train.Saver()

        print("Creating TensorFlow session...")
        sess = tf.compat.v1.Session()
        
        checkpoint_path = nets.defaultCheckpoint()
        print(f"Loading checkpoint from: {checkpoint_path}")
        
        # Check if checkpoint files exist
        import os
        checkpoint_files = [
            checkpoint_path + '.index',
            checkpoint_path + '.data-00000-of-00001',
            checkpoint_path + '.meta'
        ]
        print("Checking checkpoint files:")
        for cf in checkpoint_files:
            exists = os.path.exists(cf)
            print(f"  {cf}: {'✓' if exists else '✗'}")
            if not exists:
                print(f"    Missing file: {cf}")
        
        if not os.path.exists(checkpoint_path + '.index'):
            raise FileNotFoundError(f"Checkpoint index file not found: {checkpoint_path}.index")
            
        print("Restoring checkpoint...")
        saver.restore(sess, checkpoint_path)
        
        # Debug: Check what variables were actually loaded
        print("Variables loaded from checkpoint:")
        for var in tf.compat.v1.global_variables():
            print(f"  {var.name}: {var.shape}")
            
        print("NetVLAD initialization successful!")
        
    except Exception as e:
        print(f"NetVLAD initialization failed: {e}")
        print(f"Error type: {type(e).__name__}")
        import traceback
        traceback.print_exc()
        sess = None
        raise e


def extract(image):
    print(f"NetVLAD python extract{image.shape}")
    global image_batch
    global net_out
    global sess
    global dim

    if(image.shape[2] == 1):
        image = np.dstack((image, image, image))

    batch = np.expand_dims(image, axis=0)
    result = sess.run(net_out, feed_dict={image_batch: batch})
    
    # All that needs to be done (only valid for NetVLAD+whitening networks!)
    # to reduce the dimensionality of the NetVLAD representation below 4096 to D
    # is to keep the first D dimensions and L2-normalize.
    if(result.shape[1] > dim):
        v = result[:, :dim]
        result = v/np.linalg.norm(v)

    return np.float32(result)


if __name__ == '__main__':
    #test
    img = np.zeros([100,100,3],dtype=np.uint8)
    img.fill(255)
    init(128)
    descriptor = extract(img)
    print(descriptor.shape)
    print(descriptor)