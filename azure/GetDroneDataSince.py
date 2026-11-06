import os
import json
import logging
import re
import azure.functions as func
from azure.data.tables import TableServiceClient
from datetime import datetime, timezone

app = func.FunctionApp(http_auth_level=func.AuthLevel.ANONYMOUS)

LEGACY_YEAR = "2025"

MONTHS = {
    "January": "01",
    "February": "02",
    "March": "03",
    "April": "04",
    "May": "05",
    "June": "06",
    "July": "07",
    "August": "08",
    "September": "09",
    "October": "10",
    "November": "11",
    "December": "12",
}

ISO_DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
LEGACY_DATE_RE = re.compile(
    r"^(January|February|March|April|May|June|July|August|September|October|November|December)\s+(\d{1,2})$"
)


def is_iso_date(value):
    return isinstance(value, str) and ISO_DATE_RE.match(value)


def pretty_label_from_iso(date_iso: str) -> str:
    _, mm, dd = date_iso.split("-")
    month_name = next((name for name, num in MONTHS.items() if num == mm), mm)
    return f"{month_name} {int(dd):02d}"


def legacy_date_to_iso(date_str: str, default_year: str = LEGACY_YEAR):
    if not isinstance(date_str, str):
        return None

    m = LEGACY_DATE_RE.match(date_str.strip())
    if not m:
        return None

    month_name = m.group(1)
    day = int(m.group(2))
    month_num = MONTHS[month_name]

    return f"{default_year}-{month_num}-{day:02d}"


@app.route(route="GetDroneData")
def GetDroneData(req: func.HttpRequest) -> func.HttpResponse:
    logging.info("GetDroneData request received.")

    connection_string = os.getenv("AZURE_STORAGE_CONNECTION_STRING", "").strip()
    table_name = os.getenv("DRONE_TABLE_NAME", "").strip()

    if not connection_string:
        return func.HttpResponse("Missing AZURE_STORAGE_CONNECTION_STRING", status_code=500)

    if not table_name:
        return func.HttpResponse("Missing DRONE_TABLE_NAME", status_code=500)

    try:
        table_service = TableServiceClient.from_connection_string(connection_string)
        table_client = table_service.get_table_client(table_name)

        rows = []
        entities = table_client.list_entities()
        for e in entities:
            lat = e.get("latitude")
            lon = e.get("longitude")
            salinity = e.get("salinity")
            temperature = e.get("temperature")

            if lat in (None, 0) or lon in (None, 0):
                continue

            if salinity == -9999 and temperature == -9999:
                continue

            raw_date = e.get("Date")
            raw_time = e.get("EnclosureTime")
            sample_timestamp = e.get("SampleTimestamp")

            if raw_time and not isinstance(raw_time, str):
                raw_time = str(raw_time)

            if sample_timestamp and not isinstance(sample_timestamp, str):
                sample_timestamp = str(sample_timestamp)

            date_key = None
            date_label = None
            timestamp = None

            if is_iso_date(raw_date):
                date_key = raw_date
                date_label = pretty_label_from_iso(raw_date)

                if sample_timestamp:
                    timestamp = sample_timestamp
                elif raw_time:
                    timestamp = f"{raw_date}T{raw_time}Z"
                else:
                    timestamp = raw_date

            elif isinstance(raw_date, str) and raw_date:
                inferred_iso = legacy_date_to_iso(raw_date, LEGACY_YEAR)

                if inferred_iso:
                    date_key = inferred_iso
                    date_label = raw_date.strip()

                    if sample_timestamp:
                        timestamp = sample_timestamp
                    elif raw_time:
                        timestamp = f"{inferred_iso}T{raw_time}"
                    else:
                        timestamp = inferred_iso

            if not date_key:
                continue

            rows.append({
                "dateKey": date_key,
                "dateLabel": date_label,
                "timestamp": timestamp,
                "EnclosureTime": raw_time,

                "latitude": lat,
                "longitude": lon,
                "salinity": salinity,
                "temperature": temperature,
                "depth": e.get("depth"),
                "depthConfidence": e.get("depthConfidence"),
                "altitude": e.get("altitude"),
                "pressure": e.get("pressure"),
                "tempPressureSensor": e.get("tempPressureSensor"),

                "deviceType": e.get("deviceType", "N/A"),
                "deviceId": e.get("deviceId", "N/A"),

                "text": e.get("text"),
                "error": e.get("error", "")
            })

        return func.HttpResponse(
            json.dumps(rows),
            mimetype="application/json",
            status_code=200
        )

    except Exception as ex:
        logging.exception("Error reading Azure Table Storage.")
        return func.HttpResponse(f"Error: {str(ex)}", status_code=500)
    

@app.route(route="GetDroneDataSince")
def GetDroneDataSince(req: func.HttpRequest) -> func.HttpResponse:
    logging.info("GetDroneDataSince request received.")


    since = req.params.get("since")

    if not since:
        return func.HttpResponse(
            "Missing required query parameter: since",
            status_code=400
        )
    
    connection_string = os.getenv("AZURE_STORAGE_CONNECTION_STRING", "").strip()
    table_name = os.getenv("DRONE_TABLE_NAME", "").strip()

    if not connection_string:
        return func.HttpResponse("Missing AZURE_STORAGE_CONNECTION_STRING", status_code=500)

    if not table_name:
        return func.HttpResponse("Missing DRONE_TABLE_NAME", status_code=500)
    
    try:
        table_service = TableServiceClient.from_connection_string(connection_string)
        table_client = table_service.get_table_client(table_name)
        
        rows = []
        entities = table_client.list_entities()

        for e in entities:
            sample_timestamp = e.get("SampleTimestamp")
            raw_date = e.get("Date")
            raw_time = e.get("EnclosureTime")

            if raw_time and not isinstance(raw_time, str):
                raw_time = str(raw_time)

            effective_timestamp = None

            if isinstance(sample_timestamp, str) and sample_timestamp:
                effective_timestamp = sample_timestamp

            elif isinstance(raw_date, str) and raw_date:
                inferred_iso = legacy_date_to_iso(raw_date, LEGACY_YEAR)

                if inferred_iso:
                    if raw_time:
                        effective_timestamp = f"{inferred_iso}T{raw_time}"
                    else:
                        effective_timestamp = inferred_iso

            if not effective_timestamp:
                continue


            if effective_timestamp <= since:
                continue

            lat = e.get("latitude")
            lon = e.get("longitude")
            salinity = e.get("salinity")
            temperature = e.get("temperature")

            if lat in (None, 0) or lon in (None, 0):
                continue

            if salinity == -9999 and temperature == -9999:
                continue

            if raw_time and not isinstance(raw_time, str):
                raw_time = str(raw_time)

            date_key = None
            date_label = None
            timestamp = effective_timestamp

            if is_iso_date(raw_date):
                date_key = raw_date
                date_label = pretty_label_from_iso(raw_date)

            elif isinstance(raw_date, str) and raw_date:
                inferred_iso = legacy_date_to_iso(raw_date, LEGACY_YEAR)

                if inferred_iso:
                    date_key = inferred_iso
                    date_label = raw_date.strip()

            if not date_key:
                continue

            rows.append({
                "dateKey": date_key,
                "dateLabel": date_label,
                "timestamp": timestamp,
                "EnclosureTime": raw_time,

                "latitude": lat,
                "longitude": lon,
                "salinity": salinity,
                "temperature": temperature,
                "depth": e.get("depth"),
                "depthConfidence": e.get("depthConfidence"),
                "altitude": e.get("altitude"),
                "pressure": e.get("pressure"),
                "tempPressureSensor": e.get("tempPressureSensor"),

                "deviceType": e.get("deviceType", "N/A"),
                "deviceId": e.get("deviceId", "N/A"),

                "text": e.get("text"),
                "error": e.get("error", "")
            })

        rows.sort(key=lambda r: r["timestamp"] or "")

        return func.HttpResponse(
            json.dumps(rows),
            mimetype="application/json",
            status_code=200
        )
    
    except Exception as ex:
        logging.exception("Error reading Azure Table Storage in GetDroneDataSince.")
        return func.HttpResponse(f"Error: {str(ex)}", status_code=500)
    


@app.route(route="GetLatestDroneData")
def GetLatestDroneData(req: func.HttpRequest) -> func.HttpResponse:
    logging.info("GetLatestDroneData request received.")

    connection_string = os.getenv("AZURE_STORAGE_CONNECTION_STRING", "").strip()
    table_name = os.getenv("DRONE_TABLE_NAME", "").strip()

    if not connection_string:
        return func.HttpResponse("Missing AZURE_STORAGE_CONNECTION_STRING", status_code=500)

    if not table_name:
        return func.HttpResponse("Missing DRONE_TABLE_NAME", status_code=500)

    try:
        table_service = TableServiceClient.from_connection_string(connection_string)
        table_client = table_service.get_table_client(table_name)

        latest_row = None
        latest_timestamp = ""

        today_iso = datetime.now(timezone.utc).strftime("%Y-%m-%d")

        entities = table_client.query_entities(
            query_filter="PartitionKey eq @pk and Date eq @date",
            parameters={
                "pk": "sensor_data",
                "date": today_iso
            }
        )

        for e in entities:
            sample_timestamp = e.get("SampleTimestamp")
            raw_date = e.get("Date")
            raw_time = e.get("EnclosureTime")

            if raw_time and not isinstance(raw_time, str):
                raw_time = str(raw_time)

            effective_timestamp = None

            if isinstance(sample_timestamp, str) and sample_timestamp:
                effective_timestamp = sample_timestamp

            elif isinstance(raw_date, str) and raw_date:
                inferred_iso = legacy_date_to_iso(raw_date, LEGACY_YEAR)
                if inferred_iso:
                    if raw_time:
                        effective_timestamp = f"{inferred_iso}T{raw_time}"
                    else:
                        effective_timestamp = inferred_iso

            if not effective_timestamp:
                continue

            lat = e.get("latitude")
            lon = e.get("longitude")
            salinity = e.get("salinity")
            temperature = e.get("temperature")

            if lat in (None, 0) or lon in (None, 0):
                continue

            if salinity == -9999 and temperature == -9999:
                continue

            if effective_timestamp > latest_timestamp:
                latest_timestamp = effective_timestamp
                latest_row = e

        if not latest_row:
            return func.HttpResponse("[]", mimetype="application/json", status_code=200)

        # reuse your row formatting logic
        lat = latest_row.get("latitude")
        lon = latest_row.get("longitude")

        if lat in (None, 0) or lon in (None, 0):
            return func.HttpResponse("[]", mimetype="application/json", status_code=200)

        raw_date = latest_row.get("Date")
        raw_time = latest_row.get("EnclosureTime")

        if raw_time and not isinstance(raw_time, str):
            raw_time = str(raw_time)

        date_key = None
        date_label = None

        if is_iso_date(raw_date):
            date_key = raw_date
            date_label = pretty_label_from_iso(raw_date)

        elif isinstance(raw_date, str) and raw_date:
            inferred_iso = legacy_date_to_iso(raw_date, LEGACY_YEAR)
            if inferred_iso:
                date_key = inferred_iso
                date_label = raw_date.strip()

        row = {
            "dateKey": date_key,
            "dateLabel": date_label,
            "timestamp": latest_timestamp,
            "EnclosureTime": raw_time,

            "latitude": lat,
            "longitude": lon,
            "salinity": latest_row.get("salinity"),
            "temperature": latest_row.get("temperature"),
            "depth": latest_row.get("depth"),
            "depthConfidence": latest_row.get("depthConfidence"),
            "altitude": latest_row.get("altitude"),
            "pressure": latest_row.get("pressure"),
            "tempPressureSensor": latest_row.get("tempPressureSensor"),

            "deviceType": latest_row.get("deviceType", "N/A"),
            "deviceId": latest_row.get("deviceId", "N/A"),

            "text": latest_row.get("text"),
            "error": latest_row.get("error", "")
        }

        return func.HttpResponse(
            json.dumps([row]),
            mimetype="application/json",
            status_code=200
        )

    except Exception as ex:
        logging.exception("Error in GetLatestDroneData.")
        return func.HttpResponse(f"Error: {str(ex)}", status_code=500)
