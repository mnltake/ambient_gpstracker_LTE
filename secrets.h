//ntrip caster
const char casterHost[] = "rtk2go.com"; 
const uint16_t casterPort = 2104;
const char casterUser[] = ""; //User must provide their own email address to use RTK2Go
const char casterUserPW[] = "";
const char mountPoint[] ="mount point"; //The mount point you want to get data from

// ambient
unsigned int channelId = 100; // AmbientのチャネルID
const char* writeKey = "write key"; // ライトキー

// LTE
const char apn[] = "povo.jp";
const char gprsUser[] = "";
const char gprsPass[] = "";
