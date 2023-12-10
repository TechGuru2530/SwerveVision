package frc.robot;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class LoggerUtil {
    public static void setupLogger(Logger logger, Boolean logEnabled, String logFieStartsWith) {
        if(logEnabled){
            FileHandler fh; 
            Logger parentLog= logger.getParent();
            if (parentLog!=null&&parentLog.getHandlers().length>0) 
            {
              parentLog.removeHandler(parentLog.getHandlers()[0]);
            }
            // This block configure the logger with handler and formatter
            String directory = Paths.get("").toAbsolutePath().toString() + "/logs/";
            String fileName =  directory + logFieStartsWith + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new java.util.Date())+".log";
            try {
                Files.createDirectories(Path.of(directory));
                fh = new FileHandler(fileName);
                logger.addHandler(fh);
                SimpleFormatter formatter = new SimpleFormatter();  
                fh.setFormatter(formatter);  
        
                // the following statement is used to log any messages  
                logger.info("Log started " +  logger.getName()); 
            } catch (Exception e) {
                // TODO Auto-generated catch block
            }
        }
    }

    public static void LogInfo(Logger logger, boolean logEnabled, String logMessage) {
        if(logEnabled){
            logger.info((logMessage));
        }
    }
}
