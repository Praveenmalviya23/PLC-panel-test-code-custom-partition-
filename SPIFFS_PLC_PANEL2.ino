// BL-based LOGS PLC PANEL
/* Update Date : Oct 3 2025
   Issuer : Tushar
   Update : Bluetooth stop after 10 mins of inactivity 
            13 MB SPIFFS parition
            */

#include <BluetoothSerial.h>
#include <EEPROM.h>
#include <FS.h>
#include <SPIFFS.h>
#include "nvs_flash.h"  // Required for NVS init

#define SPIFFS_SIZE_BYTES   (13 * 1024 * 1024)  // 13 MB SPIFFS
#define SAFE_FACTOR         95                    // percent, e.g., 95%  - 5% safe margin

// Your log example
#define SAMPLE_LOG          "D:1750916400,0001,0002;"

// Number of bytes in the string literal
#define LOG_SIZE_BYTES      (sizeof(SAMPLE_LOG) - 1)  // sizeof includes null terminator

// MAX_LOGS calculation
#define MAX_LOGS            ( (int) (SPIFFS_SIZE_BYTES * SAFE_FACTOR / 100) / LOG_SIZE_BYTES)

// #define MAX_LOGS 5

// #define MAX_LOGS 10
// #define LOG_ENTRY_SIZE 26
#define LOG_ENTRY_SIZE 41
// #define LOG_FILE_PATH "/logs.txt"
#define TOLERANCE_SEC 3600
#define EEPROM_SIZE 8
#define DEVICE_ID_ADDR 0


#include "esp_spiffs.h"
#include "esp_log.h"
#define TAG "SPIFFS_ARDUINO"

#define LED 2

#define RXD2 16  // Receive pin
#define TXD2 -1  // Disable transmit to avoid conflict (shorted with RXD2)

#define Serial1_baud 115200
#define Serial2_baud 9600

unsigned long btDisconnectTime = 0;
const unsigned long BT_OFF_DELAY = 10UL * 60UL * 1000UL; // 10 minutes in ms
bool btOffScheduled = false;

HardwareSerial mySerial2(2);  // Serial2 on ESP32

BluetoothSerial SerialBT;

uint32_t lastLoggedTimestamp = 0;
int nextWriteIndex = 0;

String logFilePath = "/logs.txt";
bool espMsgSent = false;

// String testLogs[] = {
//   "D:1750916400,0001,0002#P:1750916400,01;",  // 0 - Normal
//   "D:1750916460,0003,0004#P:1750916460,02;",  // 1 - Normal
//   "D:1750916520,0005,0006#P:1750916520,03;",  // 2 - Normal
//   "D:1750916520,0005,0006#P:1750916520,03;",  // 3 - Duplicate timestamp
//   "D:1750916480,0007,0008#P:1750916480,04;",  // 4 - Older → Reset
//   "D:1750916480,0009,0010#P:1750916480,05;",  // 5 - Same timestamp → next index
//   "D:1750916580,0011,0012#P:1750916580,06;",  // 6 - Forward again
//   "D:1750916640,0013,0014#P:1750916640,07;",  // 7 - Normal
//   "D:1750916700,0015,0016#P:1750916700,08;",  // 8 - Normal
//   "D:1750916700,0015,0016#P:1750916700,08;",  // 9 - Duplicate
//   "D:1750916600,0017,0018#P:1750916600,09;",  // 10 - Older again → Reset
//   "D:1750916720,0019,0020#P:1750916720,10;",  // 11 - Forward
//   "D:1750916740,0021,0022#P:1750916740,11;",  // 12 - Normal
//   "D:1750916800,0023,0024#P:1750916800,12;",  // 13 - Normal
//   "D:1750916850,0025,0026#P:1750916850,13;",  // 14 - Normal
//   "D:1750916900,0027,0028#P:1750916900,14;",  // 15 - Normal
//   "D:1750916900,0027,0028#P:1750916900,14;",  // 16 - Duplicate
//   "D:1750916700,0029,0030#P:1750916700,15;",  // 17 - Older → Reset
//   "D:1750916920,0031,0032#P:1750916920,16;",  // 18 - Forward
//   "D:1750916950,0033,0034#P:1750916950,17;",  // 19 - Normal
//   "D:1750916950,0033,0034#P:1750916950,17;",  // 20 - Duplicate
//   "D:1750916980,0035,0036#P:1750916980,18;",  // 21 - Normal
//   "D:1750916400,0037,0038#P:1750916400,19;"   // 22 - Very old → Reset again
// };

String testLogs[] = {
  "D:1750916400,1001,2001#P:1750916400,01;",  // 0 - Normal
  "D:1750916460,1002,2002#P:1750916460,02;",  // 1 - Normal
  "D:1750916520,1003,2003#P:1750916520,03;",  // 2 - Normal
  "D:1750916580,1004,2004#P:1750916580,04;",  // 3 - Normal
  "D:1750916640,1005,2005#P:1750916640,05;",  // 4 - Normal
  "D:1750916700,1006,2006#P:1750916700,06;",  // 5 - Normal
  "D:1750916760,1007,2007#P:1750916760,07;",  // 6 - Normal
  "D:1750916820,1008,2008#P:1750916820,08;",  // 7 - Normal
  "D:1750916880,1009,2009#P:1750916880,09;",  // 8 - Normal
  "D:1750916940,1010,2010#P:1750916940,10;",  // 9 - Normal
  "D:1750917000,1011,2011#P:1750917000,11;",  // 10 - Normal
  "D:1750917060,1012,2012#P:1750917060,12;",  // 11 - Normal

  "D:1750916600,2013,3013#P:1750916600,13;",  // 12 - Older → Reset
  "D:1750916500,2014,3014#P:1750916500,14;",  // 13 - Older
  "D:1750916400,2015,3015#P:1750916400,15;",  // 14 - Older
  "D:1750916360,2016,3016#P:1750916360,16;",  // 15 - Older
  "D:1750916300,2017,3017#P:1750916300,17;"   // 16 - Older
};


const int TEST_LOG_COUNT = sizeof(testLogs) / sizeof(testLogs[0]);

void testLogsArray() {
  Serial.println("Feeding testLogs array...");
  for (int i = 0; i < TEST_LOG_COUNT; i++) {
    String formatted = formatAndPadFrame(testLogs[i]);
    if (formatted != "") {
      Serial.printf("Formatted Frame: %s\n", formatted.c_str());
      readAndPrintLogFile();
    }
    delay(100);  // simulate spaced log input
  }
}



String logs[] = {
  "D:1750916400,0001,0002;",
  "D:1750916460,0003,0004;",
  "D:1750916520,0005,0006;",
  "D:1750916580,0007,0008;",
  "D:1750916640,0009,0010;",
  "D:1750916700,0011,0012;",
  "D:1750916760,0013,0014;",
  "D:1750916820,0015,0016;",
  "D:1750916880,0017,0018;",
  "D:1750916940,0019,0020;",
  "D:1750917000,0021,0022;",
  "D:1750917060,0023,0024;",
  "D:1750917120,0025,0026;",
  "D:1750917180,0027,0028;",
  "D:1750917240,0029,0030;",
    "D:1750916400,0001,0002;",
  "D:1750916460,0003,0004;",
  "D:1750916520,0005,0006;",
  "D:1750916580,0007,0008;",
  "D:1750916640,0009,0010;",
  "D:1750916700,0011,0012;",
  "D:1750916760,0013,0014;",
  "D:1750916820,0015,0016;",
  "D:1750916880,0017,0018;",
  "D:1750916940,0019,0020;",
  "D:1750917000,0021,0022;",
  "D:1750917060,0023,0024;",
  "D:1750917120,0025,0026;",
  "D:1750917180,0027,0028;",
  "D:1750917240,0029,0030;",
    "D:1750916400,0001,0002;",
  "D:1750916460,0003,0004;",
  "D:1750916520,0005,0006;",
  "D:1750916580,0007,0008;",
  "D:1750916640,0009,0010;",
  "D:1750916700,0011,0012;",
  "D:1750916760,0013,0014;",
  "D:1750916820,0015,0016;",
  "D:1750916880,0017,0018;",
  "D:1750916940,0019,0020;",
  "D:1750917000,0021,0022;",
  "D:1750917060,0023,0024;",
  "D:1750917120,0025,0026;",
  "D:1750917180,0027,0028;",
  "D:1750917240,0029,0030;"
};

uint32_t extractTimestamp(const String& line) {
  int dStart = line.indexOf("D:");
  if (dStart == -1) return 0;
  int comma = line.indexOf(',', dStart);
  if (comma == -1) return 0;
  return line.substring(dStart + 2, comma).toInt();
}

void writeLog(uint32_t ts, const char* str) {
  // File file = SPIFFS.open(LOG_FILE_PATH, "r+");
  FILE* file = fopen("/spiffs/test.txt", "r+");
  if (!file) {
    Serial.println("❌ Failed to open log file for writing!");
    return;
  }

  int targetIndex;

  if (ts < lastLoggedTimestamp) {
    Serial.printf("♻️ Older timestamp %u < last %u. Restarting at index 0\n", ts, lastLoggedTimestamp);
    targetIndex = 0;
    nextWriteIndex = 1;
    lastLoggedTimestamp = ts;  // ✅ Fix: update to avoid multiple overwrites at 0
  } else {
    targetIndex = nextWriteIndex;
    nextWriteIndex = (nextWriteIndex + 1) % MAX_LOGS;
    lastLoggedTimestamp = ts;  // ✅ Only update if newer or same
  }

  // Format and pad log string
  String padded = String(str);
  while (padded.length() < LOG_ENTRY_SIZE - 1) padded += ' ';
  padded += '\n';

  // file.seek(targetIndex * LOG_ENTRY_SIZE);
  // file.print(padded);
  // file.close();
  fseek(file, targetIndex * LOG_ENTRY_SIZE, SEEK_SET);
  fwrite(padded.c_str(), 1, LOG_ENTRY_SIZE, file);
  fclose(file);
  Serial.printf("✅ Wrote log at index %d, timestamp %u\n", targetIndex, ts);
}


// void findLatestLogAndInit() 
// {
//   Serial.println("Finding latest Log..... ");

//   FILE* file = fopen("/spiffs/test.txt", "r");
//   if (!file) {
//     Serial.println("❌ Failed to open log file!");
//     return;
//   }

//   uint32_t latestTS = 0;
//   int latestIndex = -1;
//   char line[LOG_ENTRY_SIZE + 1];

//   for (int i = 0; i < MAX_LOGS; i++) {
//     fseek(file, i * LOG_ENTRY_SIZE, SEEK_SET);
//     if (fgets(line, sizeof(line), file)) {
//       String strLine = String(line);
//       uint32_t ts = extractTimestamp(strLine);
//       if (ts > latestTS) {
//         latestTS = ts;
//         latestIndex = i;
//       }
//     }
//   }

//   fclose(file);

//   lastLoggedTimestamp = latestTS;
//   nextWriteIndex = (latestIndex + 1) % MAX_LOGS;

//   Serial.printf("🔍 Latest log at index %d, timestamp %u\n", latestIndex, latestTS);
//   Serial.printf("🛠️ Init -> lastLoggedTimestamp = %u, nextWriteIndex = %d\n", lastLoggedTimestamp, nextWriteIndex);
// }



void findLatestLogAndInit() 
{
    Serial.println("Finding latest log...");

    FILE* file = fopen("/spiffs/test.txt", "r");
    if (!file) {
        Serial.println("❌ Failed to open log file!");
        lastLoggedTimestamp = 0;
        nextWriteIndex = 0;
        return;
    }

    // --- Step 1: Get actual file size ---
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    rewind(file);

    if (fileSize <= 0) {
        Serial.println("✅ Log file is empty.");
        fclose(file);
        lastLoggedTimestamp = 0;
        nextWriteIndex = 0;
        return;
    }

    // --- Step 2: Calculate how many logs exist ---
    int totalLogs = fileSize / LOG_ENTRY_SIZE;
    Serial.printf("📂 Max logs : %ld bytes, Logs left: %d\n", MAX_LOGS, MAX_LOGS - totalLogs);
    Serial.printf("📂 Curr File size: %ld bytes, Total Logs: %d\n\n", fileSize, totalLogs);

    char line[LOG_ENTRY_SIZE + 1];
    uint32_t latestTS = 0;
    int latestIndex = -1;

    if (totalLogs < MAX_LOGS) {
        // --- Partial file → last log only ---
        fseek(file, (totalLogs - 1) * LOG_ENTRY_SIZE, SEEK_SET);
        if (fgets(line, sizeof(line), file)) {
            latestTS = extractTimestamp(String(line));
            latestIndex = totalLogs - 1;
        }
    } else {
        // --- Full file → scan from start ---
        for (int i = 0; i < MAX_LOGS; i++) {
            fseek(file, i * LOG_ENTRY_SIZE, SEEK_SET);
            if (fgets(line, sizeof(line), file)) {
                String strLine = String(line);
                uint32_t ts = extractTimestamp(strLine);
                if (ts > latestTS) {
                    latestTS = ts;
                    latestIndex = i;
                }
            }
        }
    }

    fclose(file);

    if (latestIndex == -1) {
        Serial.println("⚠️ No valid logs found.");
        lastLoggedTimestamp = 0;
        nextWriteIndex = 0;
    } else {
        lastLoggedTimestamp = latestTS;
        nextWriteIndex = (latestIndex + 1) % MAX_LOGS;
        Serial.printf("🔍 Latest log at index %d, timestamp %u\n", latestIndex, latestTS);
        Serial.printf("🛠️ Init -> lastLoggedTimestamp = %u, nextWriteIndex = %d\n",
                      lastLoggedTimestamp, nextWriteIndex);
    }
}


void readAndPrintLogFile() {
  // File file = SPIFFS.open(LOG_FILE_PATH, "r");
  FILE* file = fopen("/spiffs/test.txt", "r");
  if (!file) {
    Serial.println("❌ Failed to open log file for reading!");
    return;
  }
  Serial.println("------ Start of Log File ------");

  char line[LOG_ENTRY_SIZE + 1];  // +1 for null terminator
  while (fgets(line, sizeof(line), file)) {
    // Remove trailing newline
    String strLine = String(line);
    strLine.trim();
    if (strLine.length() > 0) {
      Serial.println(strLine);
    }
  }

  Serial.println("------ End of Log File ------");
  fclose(file);
}


// String formatAndPadFrame(const String& frame) {
//   int dStart = frame.indexOf("D:");
//   int pStart = frame.indexOf("P:");
//   if (dStart == -1 || pStart == -1) return "";

//   String dPart = frame.substring(dStart + 2, pStart - 1);
//   int d1 = dPart.indexOf(',');
//   int d2 = dPart.indexOf(',', d1 + 1);
//   if (d1 == -1 || d2 == -1) return "";

//   String d_time = dPart.substring(0, d1);
//   String d_val1 = dPart.substring(d1 + 1, d2);
//   String d_val2 = dPart.substring(d2 + 1);

//   String pPart = frame.substring(pStart + 2, frame.indexOf(';'));
//   int p1 = pPart.indexOf(',');
//   if (p1 == -1) return "";

//   String p_time = pPart.substring(0, p1);
//   String p_val = pPart.substring(p1 + 1);

//   String output = "D:" + d_time + "," + d_val1 + "," + d_val2 + "#P:" + p_time + "," + p_val + ";";
//   writeLog(d_time.toInt(), output.c_str());
//   return output;
//  }
String formatAndPadFrame(const String& frame) {
  int dStart = frame.indexOf("D:");
  int pStart = frame.indexOf(";");

  if (dStart == -1 || pStart == -1) {
    Serial.println("⚠️ Frame missing D: or ;");
    return "";
  }

  String dPart = frame.substring(dStart + 2, pStart);
  int d1 = dPart.indexOf(',');
  int d2 = dPart.indexOf(',', d1 + 1);
  if (d1 == -1 || d2 == -1) {
    Serial.println("⚠️ Frame missing expected commas in data part");
    return "";
  }

  String d_time = dPart.substring(0, d1);
  String d_val1 = dPart.substring(d1 + 1, d2);
  String d_val2 = dPart.substring(d2 + 1);
  String output = "D:" + d_time + "," + d_val1 + "," + d_val2 + ";";

  writeLog(d_time.toInt(), output.c_str());

  return output;
}



void sendLogsOverBT() {
  FILE* f = fopen("/spiffs/test.txt", "r");
  if (!f) {
    SerialBT.println("ERROR: Log file not found.");
    return;
  }

  char line[LOG_ENTRY_SIZE + 1];  // +1 for null-terminator

  while (fgets(line, sizeof(line), f)) {
    String strLine = String(line);
    strLine.trim();  // Remove \n, \r, spaces
    if (strLine.length() > 0) {
      SerialBT.println(strLine);
      delay(100);  // Avoid Bluetooth overflow
    }
  }

  fclose(f);
  SerialBT.println("ALL_DONE");
}

void send_fake_logs()
{
  size_t num_logs = sizeof(logs) / sizeof(logs[0]);

  for (size_t i = 0; i < num_logs; i++) {
    SerialBT.println(logs[i]);
  }
}

// bool espMsgSent = false;


void handleBluetoothCommands() {
  if (!espMsgSent && SerialBT.hasClient()) 
  {
    SerialBT.println("Hi from ESP32");
    Serial.println("BLE: Device Connected");
    espMsgSent = true;
  }

  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    // if (cmd == "LED1_ON") {
    //   digitalWrite(LED, HIGH);
    //   SerialBT.println("LED turned ON");
    // }
    // else if (cmd == "LED1_OFF") {
    //   digitalWrite(LED, LOW);
    //   SerialBT.println("LED turned OFF");
    // }
    if (cmd == "BL_OFF") {
      SerialBT.println("Bluetooth turning OFF...");
      SerialBT.end();
      Serial.println("Bluetooth disabled by command.");
    } 
    else if (cmd == "GET_LOGS") 
    {
      Serial.println("BLE: GET_LOGS");
      SerialBT.println("Sending logs...");
      bool anyLineSent = false;       // flag to detect if any line was printed
      
  
      FILE* f = fopen("/spiffs/test.txt", "r");
      if (!f) {
        SerialBT.println("ERROR: Log file not found.");
        return;
      }
      char line[LOG_ENTRY_SIZE + 1];  // +1 for null-terminator

      uint32_t logs_sent_count = 0;
      while (fgets(line, sizeof(line), f)) {
        // Strip trailing newline
        int len = strlen(line);
        if (len > 0 && line[len - 1] == '\n') {
          line[len - 1] = '\0';
        }
        // Remove leading and trailing spaces
        char* start = line;
        while (*start == ' ') start++;  // skip leading spaces
        if (*start == '\0') continue;   // skip if only spaces

        anyLineSent = true;
        SerialBT.println(start);
        // Serial.println(start);
        logs_sent_count++;
        delay(1);
      }

      fclose(f);
      Serial.printf("BLE: %d logs sennt !",logs_sent_count);
      // send_fake_logs(); anyLineSent = true;     // Remove this line while not simulating
      
      if (!anyLineSent) 
      {
        Serial.println("BLE: LOG FILE EMPTY");
        SerialBT.println("LOG FILE EMPTY");
      } 
      else {
        SerialBT.println("ALL_DONE");
      }

    } 
    else if (cmd == "DELETE_LOGS") {
      Serial.println("BLE: DELETE_LOGS");
      // Call the clear file function
      clearLogFile();
      lastLoggedTimestamp = 0;
      nextWriteIndex = 0;
      SerialBT.println("LOGS Deleted Successfully !");
      delay(1);
    } else {
      SerialBT.println("Unknown command: " + cmd);
    }
  }
}


void handleUARTLine() 
{
  static String buffer = "";

  while (mySerial2.available()) {
    char c = mySerial2.read();
    Serial.print(c);
    buffer += c;

    if (c == ';') {
      buffer.trim();
      Serial.println("\nBuffer received: " + buffer);

      if (buffer.startsWith("ID:")) {
        uint32_t id = buffer.substring(3).toInt();
        Serial.println("ID RECEIVED IS:");
        Serial.println(id);

          uint32_t storedID;
        EEPROM.get(DEVICE_ID_ADDR, storedID);
        if(storedID !=  id)
        {
          Serial.println("New ID recevied | Storing & Restarting");
          EEPROM.put(DEVICE_ID_ADDR, id);
          EEPROM.commit();
          ESP.restart();
        }
        Serial.println("New & Old ID identical !");
      }

      String formatted = formatAndPadFrame(buffer);
      Serial.println("Formatted: [" + formatted + "]");

      if (formatted != "") {
        // readAndPrintLogFile();
        Serial.println("LOGGED SUCCESSFULLY");
      } else {
        Serial.println("Formatted data empty. Not logged.");
      }

      buffer = "";
    }
  }
}



// String getStoredDeviceID() {
//   uint32_t storedID;
//   EEPROM.get(DEVICE_ID_ADDR, storedID);
//   if(storedID != 0xFF ){
//   return String(storedID);
//   }
//   else{
//    return String("");
//   }
// }
String getStoredDeviceID() {
  uint32_t storedID;
  EEPROM.get(DEVICE_ID_ADDR, storedID);

  if (storedID != 0xFFFFFFFF) {
    return String(storedID);
  } else {
    return String("0");
  }
}

// void initializeLogFile() {
//   // File file = SPIFFS.open(LOG_FILE_PATH, FILE_WRITE);
//   FILE* file = fopen("/spiffs/test.txt", FILE_WRITE);
//   if (!file) {
//     Serial.println("❌ Failed to create log file!");
//     return;
//   }

//   for (int i = 0; i < 5; i++) {
//     String blankEntry = "";
//     while (blankEntry.length() < LOG_ENTRY_SIZE - 1) blankEntry += ' ';
//     blankEntry += '\n';
//     // file.print(blankEntry);
//     fwrite(blankEntry.c_str(), 1, LOG_ENTRY_SIZE, file);
//   }

//   // file.flush();  // ✅ Ensure data is written to flash
//   // file.close();
//   fclose(file);
//   Serial.println("✅ Initialized log file with blank entries.");
// }
void initializeLogFile() {
  FILE* file = fopen("/spiffs/test.txt", "w");
  if (!file) {
    Serial.println("❌ Failed to create log file!");
    return;
  }
  fclose(file);
  Serial.println("✅ Created an Empty File.");
}
// void clearLogFile() {
//   FILE* file = fopen("/spiffs/test.txt", "w");  // "w" truncates the file
//   if (!file) {
//     Serial.println("❌ Failed to clear log file!");
//     return;
//   }

//   for (int i = 0; i < MAX_LOGS; i++) {
//     String blankEntry = "";
//     while (blankEntry.length() < LOG_ENTRY_SIZE - 1) blankEntry += ' ';
//     blankEntry += '\n';
//     fwrite(blankEntry.c_str(), 1, LOG_ENTRY_SIZE, file);
//   }

//   fclose(file);
//   Serial.println("Log file cleared and overwritten with blank entries.");
// }

void clearLogFile() {
    FILE* file = fopen("/spiffs/test.txt", "w");  // truncate file
    if (!file) {
        Serial.println("❌ Failed to clear log file!");
        return;
    }
    fclose(file);
    Serial.println("Log file cleared.");
}


void setup() 
{
  Serial.begin(115200);
  mySerial2.begin(Serial2_baud, SERIAL_8N1, RXD2, TXD2);  // TXD2 = -1 disables transmit
  EEPROM.begin(EEPROM_SIZE);
  delay(2000);

  Serial.println("STARTED");

  // Step 1: Define SPIFFS config
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 2,
    .format_if_mount_failed = true
  };

  // Step 2: Register/mount SPIFFS
  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret != ESP_OK) {
    Serial.print("SPIFFS mount failed: ");
    Serial.println(esp_err_to_name(ret));
    return;
  }
  else
  {
    Serial.println("SPIFFS mounted !");
  }

  // Step 3: Get SPIFFS info
  size_t total = 0, used = 0;
  ret = esp_spiffs_info(NULL, &total, &used);
  if (ret != ESP_OK) {
    Serial.println("Failed to get SPIFFS info");
  } else {
    Serial.printf("SPIFFS Total: %d bytes, Used: %d bytes\n", total, used);
  }

  FILE* f = fopen("/spiffs/test.txt", "r");
  if (f == NULL) {
    Serial.println("⚠️ Log file missing. Creating...");
    initializeLogFile();  // This should also use fopen() internally
  }

  EEPROM.begin(512);  // Adjust size as needed

  // Dynamic Bluetooth name using stored ID
  String ch = getStoredDeviceID();
  String btName = "CBS-" + ch;

  SerialBT.begin(btName);
  Serial.println("📶 Bluetooth Started with name: " + btName + ". Waiting for client...");

  btDisconnectTime = millis();
  btOffScheduled = true;
  Serial.println("Bluetooth will be turned off after 10 mins of inactivity");

  // Init log state
  findLatestLogAndInit();
  fclose(f);
  // Enable test logs only if debugging
  // testLogsArray();  // ✅ Uncomment if you want to test logs
   Serial.println("Waiting for UART & BL commands.....");
}

void loop() 
{
  handleUARTLine();
  handleBluetoothCommands();

// Check BT connection
    if (!SerialBT.hasClient() && espMsgSent) 
    {
        espMsgSent = false;
        Serial.println("BL Device Disconnected!");

        // Start timer if not already started
        if (!btOffScheduled) {
            btDisconnectTime = millis();
            btOffScheduled = true;
            Serial.println("🔔 Disconnect timer started");
        }
    } 
    else if (SerialBT.hasClient()) 
    {
        // Reset timer if BT reconnects
        if (btOffScheduled) {
            btOffScheduled = false;
            Serial.println("🔄 BT reconnected, timer reset");
        }
    }

    // Check if timer expired
    if (btOffScheduled && (millis() - btDisconnectTime >= BT_OFF_DELAY)) 
    {
        btOffScheduled = false;
        Serial.println("⏱️ 10 minutes passed, turning off Bluetooth...");
        SerialBT.end(); // turn off BT
    }

}