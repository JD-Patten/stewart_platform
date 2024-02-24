import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import csv

class tf_msg_info():
    def __init__(self,time,frame_id,child_frame_id,translation,rotation):
        self.time = time * 0.000000001
        self.frame = frame_id
        self.child_frame = child_frame_id
        self.translation = translation
        self.rotation = rotation

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]
    
    def save_messages(self, topic_name, csv_filename):
        messages = self.get_messages(topic_name)

        tfs = []
        for message in messages:


            time = message[0]
            frame_id = message[1].header.frame_id

            child_frame_id = message[1].child_frame_id
            
            translation_vector = (message[1].transform.translation.x, 
                                    message[1].transform.translation.y, 
                                    message[1].transform.translation.z)
        
            quaternion = (message[1].transform.rotation.x, 
                            message[1].transform.rotation.y, 
                            message[1].transform.rotation.z,
                            message[1].transform.rotation.w)
            
            tfs.append(tf_msg_info(time, frame_id, child_frame_id, translation_vector, quaternion))

        with open(csv_filename, mode="w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            for tf in tfs:
                writer.writerow([
                    tf.time,
                    tf.frame,
                    tf.child_frame,
                    tf.translation[0],
                    tf.translation[1],
                    tf.translation[2],
                    tf.rotation[0],
                    tf.rotation[1],
                    tf.rotation[2],
                    tf.rotation[3],
                ])


        return tfs

        


if __name__ == "__main__":

    bag_file = "/home/jd/Workspaces/robot_ws/src/robot/bag_files/rosbag2_2024_01_21-12_03_13/rosbag2_2024_01_21-12_03_13_0.db3"
    

    parser = BagFileParser(bag_file)

    tfs = parser.save_messages("/blender_tfs", 'static_245z.csv')

