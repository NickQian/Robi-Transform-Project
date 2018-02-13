#!/usr/bin/env python

"""
 Thing recognize bases on tensorflow demo.
 ----
 Licensed under BSD license. See the LICENSE file in the root.
 0.1:  2016-05-07. init by by Nick Qian
 ----
 input: cam shots
 output: string
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os.path
import re
import sys
import tarfile
import numpy as np
#from six.moves import urllib
import tensorflow as tf

FLAGS= tf.app.flags.FLAGS

tf.app.flags.DEFINE_string('model_dir', '/home/pi/toby_ws/data',
   """path to <classify_image_graph_def.pb>, <imagenet_synset_to_human_label_map.txt> and <imagenet_2012_channenge_label_map_proto.pbtxt> """    )
tf.app.flags.DEFINE_string('image_file',   '/home/pi/toby_ws/data/pangxie_7.jpg',    #  grace_hopper.jpg cropped_panda.jpg
                           """ Absolute path to image file """)
tf.app.flags.DEFINE_integer('num_top_predictions', 5,
                            """ Display this many predictions """)

class NodeLookup(object):
    """ convert int ID to label string"""
    def __init__(self, label_lookup_path=None, uid_lookup_path=None):          #UID(string) to int node ID,   UID to readable string 
        if label_lookup_path == None:
            label_lookup_path = os.path.join(FLAGS.model_dir,  'imagenet_2012_challenge_label_map_proto.pbtxt')
        if uid_lookup_path == None:
            uid_lookup_path = os.path.join(FLAGS.model_dir,  'imagenet_synset_to_human_label_map.txt' )  # 'imagenet_comp_graph_label_strings.txt'
        self.node_lookup = self.load(label_lookup_path, uid_lookup_path)

    def load(self, label_lookup_path, uid_lookup_path):
        if not tf.gfile.Exists(uid_lookup_path):
            tf.logging.fatal('Error: File %s does not exist! ', uid_lookup_path)
        if not tf.gfile.Exists(label_lookup_path):
            tf.logging.fatal('Error: File %s does not exist!', label_lookup_path)

        #---------UID(string) to readable string
        proto_as_ascii_lines = tf.gfile.GFile(uid_lookup_path).readlines()
        uid_to_human = { }
        p = re.compile(r'[n\d]*[ \S,]*')
        for line in proto_as_ascii_lines:
            parsed_items = p.findall(line)
            uid = parsed_items[0]
            human_string = parsed_items[2]
            uid_to_human[uid] = human_string
        #------- UID(string) to int node ID
        node_id_to_uid = { }
        proto_as_ascii = tf.gfile.GFile(label_lookup_path).readlines()
        for line in proto_as_ascii:
            if line.startswith('  target_class:'):
                target_class = int(line.split(': ')[1])
            if line.startswith('  target_class_string:'):
                target_class_string = line.split(': ')[1]
                node_id_to_uid[target_class] = target_class_string[1:-2]
        #-------- int node ID to readable string
        node_id_to_name = { }
        for key, val in node_id_to_uid.items():
            if val not in uid_to_human:
                tf.logging.fatal('Error: Failed to locate: %s', val)
            name = uid_to_human[val]
            node_id_to_name[key] = name

        return node_id_to_name

    def id_to_string(self, node_id):
        if node_id not in self.node_lookup:
            return ''
        return self.node_lookup[node_id]
#----------------
#/home/pi/toby_ws/data/imagenet_comp_graph_label_strings.txt   tensorflow_inception_graph.pb
def create_graph( ):
    with tf.gfile.FastGFile(os.path.join(FLAGS.model_dir, 'classify_image_graph_def.pb'), 'rb') as f:        #'classify_image_graph_def.pb'  tensorflow_inception_graph.pb
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read( ) )
        _ = tf.import_graph_def(graph_def, name = '' )

def thingRec(image):
    # tf.app.flags.DEFINE_string('image_file',   img)   #  '/home/pi/toby_ws/data/pangxie_7.jpg'       "" Absolute path to image file """
    if not tf.gfile.Exists(image):
        tf.logging.fatal('Error: File %s does not exost', image)
    image_data = tf.gfile.FastGFile(image, 'rb').read( )

    #Creates graph from saved GrapgDef
    create_graph( )

    with tf.Session() as sess:      # 'softmax:0': A tensor contains normalized prediction; 'pool_3:0'; 'DecodeJpeg/contents:0'
        softmax_tensor = sess.graph.get_tensor_by_name('softmax:0')
        '''--------------------------------------------------------------------------------------------------------------------------------------------------
            Is it possible to extract the obj detection info here and pass it to the high level node vp?
            tf.scalar_summary((tags, values, collections=None, name=None)                   Outputs a `Summary` protocol buffer with scalar values.  
            tf.histogram_summary(tag, values, collections=None, name=None )              Outputs a `Summary` protocol buffer with a histogram.
            tf.Summary
            tf.image_summary(tag, tensor, max_images=3, collections=None, name=None)      Outputs a `Summary` protocol buffer with images
            tf.merge_summary()
           -----------------------------------------------------------------------------------------------------------------------------------------------------'''
        predictions = sess.run(softmax_tensor, {'DecodeJpeg/contents:0': image_data} )
        print ('~~~~sess_str is: ', sess.sess_str, '~~~~target is:', sess._target)

        predictions = np.squeeze(predictions)

    # node ID -> string lookup
    node_lookup = NodeLookup( )

    top_k = predictions.argsort()[-FLAGS.num_top_predictions:][::-1]
    print ('$$$:top_k is:', top_k)
    rsltdic = { }
    for node_id in top_k:
        human_string = node_lookup.id_to_string(node_id)
        score = predictions[node_id]
        rsltdic[int(score*1000)] = human_string
        #print('$$$ Info:%s(score = %.3f)' % (human_string, score ) )

    result = sorted(rsltdic.items(), key = lambda d:d[0], reverse = True)
    print ('@@@:', result)
    return result


def main(_):
    image = (FLAGS.image_file if FLAGS.image_file else os.path.join(FLAGS.model_dir, 'cropped_panda.jpg' ))
    print ('Info: image input is:', image)
    thingRec(image)

    
if __name__ == '__main__':
    #tf.app.run( )
    thingRec( '/home/pi/toby_ws/data/pangxie_7.jpg' )
    
