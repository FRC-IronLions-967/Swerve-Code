/**
 * Java class to read values from Java .properties files for use in FRC robotics.
 */

 package frc.robot.Utils;

 import java.io.*;
 import java.util.Properties;
 
 public class Values {
 
     private static Values inst = null;
     private static final String[] FILE_PATHS = {"/home/lvuser/deploy/robot.properties"};
 
     private Properties[] properties;
 
     private Values(String[] paths) {
         properties = new Properties[FILE_PATHS.length];
 
         for(int i = 0; i < paths.length; i++) {
            try {
                File file = new File(FILE_PATHS[i]);
                FileInputStream in = new FileInputStream(file);
                properties[i] = new Properties();
                properties[i].load(in);
            } catch (IOException e) {
                e.printStackTrace();
            }
         }
     }
 
     public static Values getInstance() {
         if(inst == null) inst = new Values(FILE_PATHS);
 
         return inst;
     }
 
     public String getStringValue(String key) {
         for(Properties p : properties) {
            if(p == null) continue;
             String s = p.getProperty(key);
             if(s != null) return s;
         }
 
         return null;
     }
 
     public Double getDoubleValue(String key) {
         for(Properties p : properties) {
            if(p == null) continue;
             String s = p.getProperty(key);
             if(s != null) return Double.parseDouble(s);
         }
 
         return null;
     }
 
     public Integer getIntValue(String key) {
         for(Properties p : properties) {
            if(p == null) continue;
             String s = p.getProperty(key);
             if(s != null) return Integer.parseInt(s);
         }
 
         return null;
     }
 
     public Boolean getBooleanValue(String key) {
         for(Properties p : properties) {
            if(p == null) continue;
             String s = p.getProperty(key);
             if(s != null) return Boolean.parseBoolean(s);
         }
 
         return null;
     }
 
 }