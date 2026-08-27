// Minimal XML-RPC client for the Fairino controller's peripheral API on TCP
// 20003. This is the transport the vendor SDK (fairino-cpp-sdk) uses for
// gripper control (ActGripper/MoveGripper/GetGripperConfig/...), which is not
// available over the documented 8080 command protocol the arm uses — so the
// gripper component (gripper.go) talks XML-RPC while the arm stays on 8080.
//
// Only the small slice of XML-RPC the controller actually speaks is
// implemented: methodCall with int/double/string params, and methodResponse
// decoding of scalars, <array>, and <struct> (for <fault>). Every Fairino
// method returns an array whose first element is an int errcode (0 == ok).
package viamfairino

import (
	"bytes"
	"context"
	"encoding/xml"
	"fmt"
	"io"
	"net/http"
	"strconv"
	"strings"
	"time"
)

const xmlrpcPort = 20003

type xmlrpcClient struct {
	url  string
	http *http.Client
}

func newXMLRPCClient(host string, port int) *xmlrpcClient {
	return &xmlrpcClient{
		url:  fmt.Sprintf("http://%s:%d/RPC2", host, port),
		http: &http.Client{Timeout: 10 * time.Second},
	}
}

// call invokes method with the given params (int, float64, or string) and
// returns the response array flattened to []any. A <fault> response is
// returned as an error.
func (c *xmlrpcClient) call(ctx context.Context, method string, params ...any) ([]any, error) {
	reqBody, err := encodeXMLRPCRequest(method, params)
	if err != nil {
		return nil, err
	}
	req, err := http.NewRequestWithContext(ctx, http.MethodPost, c.url, bytes.NewReader(reqBody))
	if err != nil {
		return nil, err
	}
	req.Header.Set("Content-Type", "text/xml")
	resp, err := c.http.Do(req)
	if err != nil {
		return nil, fmt.Errorf("xmlrpc %s: %w", method, err)
	}
	defer func() { _ = resp.Body.Close() }()
	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("xmlrpc %s: reading response: %w", method, err)
	}
	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("xmlrpc %s: HTTP %d: %s", method, resp.StatusCode, strings.TrimSpace(string(body)))
	}
	vals, err := parseXMLRPCResponse(body)
	if err != nil {
		return nil, fmt.Errorf("xmlrpc %s: %w", method, err)
	}
	return vals, nil
}

// ControllerError is a non-zero errcode returned by a Fairino XML-RPC method.
// Callers can inspect Code (via errors.As) to react to specific controller
// errors, e.g. errcode 73 ("gripper movement error").
type ControllerError struct {
	Method string
	Code   int
}

func (e *ControllerError) Error() string {
	return fmt.Sprintf("xmlrpc %s: controller errcode %d", e.Method, e.Code)
}

// callChecked calls method and verifies the leading int errcode is 0,
// returning the trailing values (everything after the errcode). A non-zero
// errcode is returned as a *ControllerError.
func (c *xmlrpcClient) callChecked(ctx context.Context, method string, params ...any) ([]any, error) {
	vals, err := c.call(ctx, method, params...)
	if err != nil {
		return nil, err
	}
	if len(vals) == 0 {
		return nil, fmt.Errorf("xmlrpc %s: empty response", method)
	}
	code, ok := vals[0].(int)
	if !ok {
		return nil, fmt.Errorf("xmlrpc %s: leading value %v (%T) is not an errcode", method, vals[0], vals[0])
	}
	if code != 0 {
		return nil, &ControllerError{Method: method, Code: code}
	}
	return vals[1:], nil
}

func encodeXMLRPCRequest(method string, params []any) ([]byte, error) {
	var b strings.Builder
	b.WriteString(`<?xml version="1.0"?><methodCall><methodName>`)
	b.WriteString(method)
	b.WriteString(`</methodName><params>`)
	for _, p := range params {
		b.WriteString("<param><value>")
		switch v := p.(type) {
		case int:
			b.WriteString("<i4>" + strconv.Itoa(v) + "</i4>")
		case float64:
			b.WriteString("<double>" + strconv.FormatFloat(v, 'f', -1, 64) + "</double>")
		case string:
			b.WriteString("<string>" + xmlEscape(v) + "</string>")
		default:
			return nil, fmt.Errorf("unsupported xmlrpc param type %T", p)
		}
		b.WriteString("</value></param>")
	}
	b.WriteString("</params></methodCall>")
	return []byte(b.String()), nil
}

func xmlEscape(s string) string {
	var buf bytes.Buffer
	_ = xml.EscapeText(&buf, []byte(s))
	return buf.String()
}

// parseXMLRPCResponse returns the response's leading array flattened to []any,
// or an error for a <fault>.
func parseXMLRPCResponse(data []byte) ([]any, error) {
	dec := xml.NewDecoder(bytes.NewReader(data))
	faulted := false
	for {
		tok, err := dec.Token()
		if err == io.EOF {
			break
		}
		if err != nil {
			return nil, err
		}
		se, ok := tok.(xml.StartElement)
		if !ok {
			continue
		}
		switch se.Name.Local {
		case "fault":
			faulted = true
		case "value":
			v, err := decodeXMLRPCValue(dec)
			if err != nil {
				return nil, err
			}
			if faulted {
				return nil, fmt.Errorf("xmlrpc fault: %v", v)
			}
			if arr, ok := v.([]any); ok {
				return arr, nil
			}
			return []any{v}, nil
		}
	}
	return nil, fmt.Errorf("no <value> in response: %s", strings.TrimSpace(string(data)))
}

// decodeXMLRPCValue decodes one value; the decoder must be positioned just
// after the value's <value> start element. It does not consume the closing
// </value>.
func decodeXMLRPCValue(dec *xml.Decoder) (any, error) {
	for {
		tok, err := dec.Token()
		if err != nil {
			return nil, err
		}
		switch t := tok.(type) {
		case xml.StartElement:
			switch t.Name.Local {
			case "i4", "int":
				return decodeInt(dec, &t)
			case "double":
				var s string
				if err := dec.DecodeElement(&s, &t); err != nil {
					return nil, err
				}
				return strconv.ParseFloat(strings.TrimSpace(s), 64)
			case "boolean":
				var s string
				if err := dec.DecodeElement(&s, &t); err != nil {
					return nil, err
				}
				return strings.TrimSpace(s) == "1", nil
			case "string":
				var s string
				if err := dec.DecodeElement(&s, &t); err != nil {
					return nil, err
				}
				return s, nil
			case "array":
				return decodeXMLRPCArray(dec)
			case "struct":
				return decodeXMLRPCStruct(dec)
			}
		case xml.CharData:
			if s := strings.TrimSpace(string(t)); s != "" {
				return s, nil // untyped value == string
			}
		case xml.EndElement:
			if t.Name.Local == "value" {
				return "", nil // empty <value/>
			}
		}
	}
}

func decodeInt(dec *xml.Decoder, start *xml.StartElement) (any, error) {
	var s string
	if err := dec.DecodeElement(&s, start); err != nil {
		return nil, err
	}
	return strconv.Atoi(strings.TrimSpace(s))
}

func decodeXMLRPCArray(dec *xml.Decoder) ([]any, error) {
	out := []any{}
	for {
		tok, err := dec.Token()
		if err != nil {
			return nil, err
		}
		switch t := tok.(type) {
		case xml.StartElement:
			if t.Name.Local == "value" {
				v, err := decodeXMLRPCValue(dec)
				if err != nil {
					return nil, err
				}
				out = append(out, v)
			}
		case xml.EndElement:
			if t.Name.Local == "array" {
				return out, nil
			}
		}
	}
}

func decodeXMLRPCStruct(dec *xml.Decoder) (map[string]any, error) {
	out := map[string]any{}
	var name string
	for {
		tok, err := dec.Token()
		if err != nil {
			return nil, err
		}
		switch t := tok.(type) {
		case xml.StartElement:
			switch t.Name.Local {
			case "name":
				if err := dec.DecodeElement(&name, &t); err != nil {
					return nil, err
				}
			case "value":
				v, err := decodeXMLRPCValue(dec)
				if err != nil {
					return nil, err
				}
				out[name] = v
			}
		case xml.EndElement:
			if t.Name.Local == "struct" {
				return out, nil
			}
		}
	}
}
